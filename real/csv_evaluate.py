import torch
import numpy as np
import pandas as pd 
import random
import rospy
import pickle
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time 
import readchar
import sys
import pandas as pd 
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))

from controller.robot_state import *
from panda_kinematics import PandaWithPumpKinematics
from settings.var import GRIPPER_FORCE, BOX_Z, POLICY_CONFIG, TASK_CONFIG, TRAIN_CONFIG
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from rich.progress import Progress
from training.utils import make_policy, get_image

CSV_DIR = "/home/summit-nuc/Real_ACTFranka/real_dir3/csv" 

class Simulator:
    def __init__(self, policy, policy_config, task_config, stats, device):

        self.franka = RobotController()
        self.policy = policy
        self.policy_config = policy_config
        self.cfg = task_config
        self.device = device
        self.stats = stats
        self.pre_process = lambda pos: (pos - stats['qpos_mean']) / stats['qpos_std']
        self.post_process = lambda act: act * stats['action_std'] + stats['action_mean']
        self.bridge = CvBridge()
        self.franka.initial_pose()        
        self.gripper_grasp_position = False


    def set_box_position(self, x, y, z):
            rospy.wait_for_service('/gazebo/set_model_state')
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            state_msg = SetModelStateRequest()
            state_msg.model_state.model_name = 'stone'
            state_msg.model_state.pose.position.x = x
            state_msg.model_state.pose.position.y = y
            state_msg.model_state.pose.position.z = z
            set_state(state_msg)

    def generate_coordinate(self):
            box_length = 0.18
            box_width = 0.11
            box_x_center = 0.45
            box_y_center = -0.21
            cube_x = 0.025
            cube_y = 0.032
            min_x = box_x_center - box_length / 2 + cube_x / 2
            max_x = box_x_center + box_length / 2 - cube_x / 2
            min_y = box_y_center - box_width / 2 + cube_y / 2
            max_y = box_y_center + box_width / 2 - cube_y / 2
            
            x = random.uniform(min_x, max_x)
            y = random.uniform(min_y, max_y)

            return x , y

    def capture_image(self, camera_name):
        camera_topics = {
            'left': '/multisense_sl/camera/left/image_raw',
            'right': '/multisense_sl/camera/right/image_raw',
            'gripper': '/gripper_camera/image_raw'
        }

        topic = camera_topics.get(camera_name, '/multisense_sl/camera/left/image_raw') 
        msg = rospy.wait_for_message(topic, Image, timeout=5)
        return self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def simulate(self):
        query_frequency = self.policy_config['num_queries'] #149 
        
        # ====================== need to change real csv file name ============================
        df_actions = pd.read_csv(os.path.joint(CSV_DIR,'episode_25_action.csv')) 
        action_len = len(df_actions)

        # ====================== need to change real csv file name ============================


        qpos_history = torch.zeros((1, self.cfg['episode_len'], self.cfg['state_dim'])).to(self.device)

        with Progress() as progress:

            # task = progress.add_task("[green]Running real-time simulation...", total=self.cfg['episode_len'])
            action_list = []
            
            print("Press Enter 'n' to go to the next step (or 'q' to quit)")
            
            t = 0

            while t < action_len:
                
                key = readchar.readkey()
                print(f"[INFO] Step {t} Running next step ...  ")

                if key == 'n':

                    # progress.update(task, advance=1)

                    qpos = self.franka.angles()
                    gripper_width = self.franka.gripper_state()
                    gripper_binary = 0 if gripper_width > 0.04 else 1
                    pos = np.append(qpos, gripper_binary)

                    images = {cn: self.capture_image(cn) for cn in self.cfg['camera_names']}
                    qpos_input = torch.from_numpy(self.pre_process(pos)).float().to(self.device).unsqueeze(0)
                    qpos_history[:, t] = qpos_input
                    curr_image = get_image(images, self.cfg['camera_names'], self.device)
                        
                    if self.policy_config['temporal_agg']:
                        all_time_actions = df_actions
                        actions_for_curr_step = all_time_actions.iloc[t].to_numpy() 
                   

                    action = actions_for_curr_step
                    # Apply action to robot
                    self.franka.current_positions = self.franka.angles()

                    joint_positions = action[:7]
                    #print(joint_positions)
                    gripper_action = action[7]
                    self.franka.move_to_joint_position(joint_positions)

                    print("gripper action: ", gripper_action)
                    
                    if not self.gripper_grasp_position:

                        if gripper_action > 0.2: # 0.9
                            print("IN gripper action !!!!!!!!! ")
                            self.franka.grasp(0.009, 42)
                            self.gripper_grasp_position = True

                        else:

                            print("in else grippger aciton !!!!!!!!!! ")
                            self.franka.exec_gripper_cmd(0.08, GRIPPER_FORCE)
                    t+=1

                elif key == 'q': 
                    print("Exiting simulation...")
                    break        
            df = pd.DataFrame(action_list, columns = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "gripper"])
            cur_time = time()
            df.to_csv(f'actions_{cur_time}.csv', index=False)

if __name__ == "__main__":
    #rospy.init_node("real_time_simulator")

    cfg = TASK_CONFIG
    policy_config = POLICY_CONFIG
    train_cfg = TRAIN_CONFIG
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

    # Load policy
    policy = make_policy(policy_config['policy_class'], policy_config)
    ckpt_path = os.path.join(train_cfg['checkpoint_dir'], train_cfg['eval_ckpt_name'])
    policy.load_state_dict(torch.load(ckpt_path, map_location=device))
    policy.to(device)
    policy.eval()

    # Load stats
    stats_path = os.path.join(train_cfg['checkpoint_dir'], 'dataset_stats.pkl')
    with open(stats_path, 'rb') as f:
        stats = pickle.load(f)

    simulator = Simulator(policy, policy_config, cfg, stats, device)
    box_x, box_y = simulator.generate_coordinate()
    simulator.set_box_position(box_x, box_y, BOX_Z)
    rospy.sleep(1)
    simulator.simulate()
