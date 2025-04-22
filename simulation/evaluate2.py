import torch
import gc
import numpy as np
import pandas as pd 
import random
import rospy
import pickle
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))

from controller.robot_state import *
from panda_kinematics import PandaWithPumpKinematics
from settings.var import GRIPPER_FORCE, BOX_Z, POLICY_CONFIG, TASK_CONFIG, TRAIN_CONFIG
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from rich.progress import Progress
from training.utils import make_policy, get_image

def clear_cuda():
    torch.cuda.empty_cache()
    gc.collect()
    print("기억 삭제 기억 삭제 기억 삭제 기억 삭제 기억 삭제 기억 삭제 기억 삭제 기억 삭제 기억 삭제 기억 삭제 기억 삭제 기억 삭제 ")

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
            
            x = random.uniform(min_x + 0.02, max_x - 0.02)
            y = random.uniform(min_y, max_y - 0.05)

            return x , y

    def capture_image(self, camera_name):
        camera_topics = {
            'left': '/multisense_sl/camera/left/image_raw',
            'right': '/multisense_sl/camera/right/image_raw',
            'gripper': '/gripper_camera/image_raw'
        }

        # =============================================== need to check ==========================================
        topic = camera_topics.get(camera_name, '/multisense_sl/camera/left/image_raw') 
        # =============================================== need to check ==========================================

        msg = rospy.wait_for_message(topic, Image, timeout=5)
        return self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def simulate(self):
        query_frequency = self.policy_config['num_queries'] #149 
        
        if self.policy_config['temporal_agg']:
            query_frequency = 10 # 30  
            num_queries = self.policy_config['num_queries']
            all_time_actions = torch.zeros([self.cfg['episode_len'], self.cfg['episode_len']+num_queries, self.cfg['state_dim']]).to(self.device)

        qpos_history = torch.zeros((1, self.cfg['episode_len'], self.cfg['state_dim'])).to(self.device)

        with Progress() as progress:
            task = progress.add_task("[green]Running real-time simulation...", total=self.cfg['episode_len'])
            action_list = []
            for t in range(self.cfg['episode_len']):

                progress.update(task, advance=1)

                qpos = self.franka.angles()
                gripper_width = self.franka.gripper_state()
                gripper_binary = 0 if gripper_width > 0.04 else 1
                pos = np.append(qpos, gripper_binary)

                images = {cn: self.capture_image(cn) for cn in self.cfg['camera_names']}
                qpos_input = torch.from_numpy(self.pre_process(pos)).float().to(self.device).unsqueeze(0)
                #qpos_history[:, t] = qpos_input
                curr_image = get_image(images, self.cfg['camera_names'], self.device)

                if t % query_frequency == 0:
                    print("[INFO] Running policy..!!!!!!!!!!!!!.")
                    with torch.no_grad():
                        all_actions = self.policy(qpos_input, curr_image)

                    
                if self.policy_config['temporal_agg']:
                    all_time_actions[[t], t:t+num_queries] = all_actions
                    actions_for_curr_step = all_time_actions[:, t]
                    actions_populated = torch.all(actions_for_curr_step != 0, axis=1)
                    actions_for_curr_step = actions_for_curr_step[actions_populated]
                    k = 0.01
                    exp_weights = np.exp(-k * np.arange(len(actions_for_curr_step)))
                    exp_weights = exp_weights / exp_weights.sum()
                    exp_weights = torch.from_numpy(exp_weights.astype(np.float32)).to(self.device).unsqueeze(dim=1)
                    raw_action = (actions_for_curr_step * exp_weights).sum(dim=0, keepdim=True)
                else:
                    raw_action = all_actions[:, t % query_frequency]

                action = self.post_process(raw_action.squeeze(0).detach().cpu().numpy())
                #action_list.append(action.tolist()) # Convert numpy array to list before appending 
                # Apply action to robot
                self.franka.current_positions = self.franka.angles()

                joint_positions = action[:7]
                #print(joint_positions)
                gripper_action = action[7]

                position_error = np.abs(np.array(joint_positions) - np.array(self.franka.current_positions))
                # 허용 오차보다 크면 바로 포지션으로 이동
                if np.any(position_error[3:] > 0.7):  # threshold는 상황에 맞게 조정
                    joint_positions = self.franka.current_positions
                    rospy.logwarn(f"Position error too large: {position_error}")

                #if t % 50 == 0:
                #self.franka.move_to_joint_position(joint_positions)
                self.franka.move_to_joint_position_with_PID(
                        joint_positions
                    )
                    ##

                print("gripper action: ", gripper_action)
                
                if not self.gripper_grasp_position:

                    if gripper_action > 0.4: # 0.9
                        print("IN gripper action !!!!!!!!! ")
                        self.franka.grasp(0.0399, 5)
                        #self.gripper_grasp_position = True

                    else:
                        print("in else grippger aciton !!!!!!!!!! ")
                        self.franka.exec_gripper_cmd(0.08, GRIPPER_FORCE)

                #if t % 130 == 0 and t > 200 :  # 5 step마다 GPU 캐시 비우기
                    #del all_actions, qpos_input, curr_image, raw_action
                    #clear_cuda()

            df = pd.DataFrame(action_list, columns = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "gripper"])
            df.to_csv('actions_0414_1418.csv', index=False)

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

