import torch
import numpy as np
import pandas as pd 
import random
import rospy
import pickle
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from datetime import datetime
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))

from settings.var import GRIPPER_FORCE, BOX_Z, POLICY_CONFIG, TASK_CONFIG, TRAIN_CONFIG
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from rich.progress import Progress
from training.utils import make_policy, get_image

import readchar 
from datetime import datetime 
import actionlib
from std_msgs.msg import Float64MultiArray
from franka_gripper.msg import MoveAction, MoveGoal, GraspAction, GraspGoal, GraspEpsilon   


DATASET_DIR = "/home/summit-nuc/Real_ACTFranka/real_dir3/processed"
CSV_DIR = "/home/summit-nuc/Real_ACTFranka/real_dir3/csv" 
OPEN_GRIPPER_POSE = 0.08 
CLOSE_GRIPPER_POSE = 0.03 

class Simulator:
    def __init__(self, policy, policy_config, task_config, stats, device):
        rospy.init_node('robot_controller', anonymous=True)
        self.current_joint_angles = [0.0] * 7
        self.current_gripper_width = OPEN_GRIPPER_POSE 
        self.joint_state_sub = rospy.Subscriber('/franka_state_controller/joint_states', JointState, self.joint_state_callback)
        self.gripper_state_sub = rospy.Subscriber('/franka_gripper/joint_states', JointState, self.gripper_state_callback)
        self.pub = rospy.Publisher('/joint_group_position_controller/command',
                                   Float64MultiArray, queue_size=1)
        
        self.move_cli = actionlib.SimpleActionClient("/franka_gripper/move", MoveAction)
        self.grasp_cli = actionlib.SimpleActionClient("/franka_gripper/grasp", GraspAction)
        
        rospy.loginfo("Waiting for gripper servers…"); self.move_cli.wait_for_server(); self.grasp_cli.wait_for_server()

        self.policy = policy
        self.policy_config = policy_config
        self.cfg = task_config
        self.device = device
        self.stats = stats
        self.pre_process = lambda pos: (pos - stats['qpos_mean']) / stats['qpos_std']
        self.post_process = lambda act: act * stats['action_std'] + stats['action_mean']
        self.bridge = CvBridge()
        self.gripper_grasp_position = False

        self.joint_positions = []
        self.joint_received = False
        self.gripper_received = False

    def joint_state_callback(self, msg):
        self.current_joint_angles = list(msg.position[:7])

    def gripper_state_callback(self, msg):
        self.current_gripper_width = msg.position[0]

    def capture_image(self, camera_name):
        camera_topics = {

            'left': '/camera1/color/image_raw',
            'right': '/camera2/color/image_raw',
            'gripper': '/camera3/color/image_raw'
        }

        # =============================================== need to check ==========================================
        topic = camera_topics.get(camera_name, '/camera1/color/image_raw') 
        # =============================================== need to check ==========================================

        msg = rospy.wait_for_message(topic, Image, timeout=5)
        return self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def simulate(self):
        
        query_frequency = self.policy_config['num_queries'] #149 
        
        if self.policy_config['temporal_agg']:
            num_queries = self.policy_config['num_queries']
            all_time_actions = torch.zeros([self.cfg['episode_len'], self.cfg['episode_len']+num_queries, self.cfg['state_dim']]).to(self.device)

        qpos_history = torch.zeros((1, self.cfg['episode_len'], self.cfg['state_dim'])).to(self.device)

        with Progress() as progress:
            
            task = progress.add_task("[green]Running real-time simulation...", total=self.cfg['episode_len'])
            action_list = []
            t = 0 

            while t < self.cfg['episode_len']:
                key = readchar.readkey()
                print(f"[Progress] {t} / {self.cfg['episode_len']} Steps")
                
                if key == ' ': 
                    progress.update(task, advance=1)
      
                    qpos = self.current_joint_angles
                    gripper_width = self.current_gripper_width 
                    gripper_binary = 0 if gripper_width > 0.04 else 1
                    pos = np.append(qpos, gripper_binary)

                    images = {cn: self.capture_image(cn) for cn in self.cfg['camera_names']}
                    qpos_input = torch.from_numpy(self.pre_process(pos)).float().to(self.device).unsqueeze(0)
                    qpos_history[:, t] = qpos_input
                    curr_image = get_image(images, self.cfg['camera_names'], self.device)

                    if t % num_queries == 0:
                        print("[INFO] Running policy")
                        with torch.no_grad():
                            all_actions = self.policy(qpos_input, curr_image)

                        
                    if self.policy_config['temporal_agg']:
                        all_time_actions[[t], t:t+num_queries] = all_actions
                        actions_for_curr_step = all_time_actions[:, t]
                        actions_populated = torch.all(actions_for_curr_step != 0, axis=1) # True or False 
                        actions_for_curr_step = actions_for_curr_step[actions_populated]
                        
                        k = 0.01
                        exp_weights = np.exp(-k * np.arange(len(actions_for_curr_step)))
                        exp_weights = exp_weights / exp_weights.sum()
                        exp_weights = torch.from_numpy(exp_weights.astype(np.float32)).to(self.device).unsqueeze(dim=1)
                        raw_action = (actions_for_curr_step * exp_weights).sum(dim=0, keepdim=True)

                    else:
                        raw_action = all_actions[:, t % query_frequency]

                    action = self.post_process(raw_action.squeeze(0).detach().cpu().numpy())
                    action_list.append(action.tolist()) # Convert numpy array to list before appending 
                
                    joint_positions = action[:7]
                    gripper_action = action[7]

                    print("[INFO] grippper action : ", gripper_action)
                    print("[INFO] joint action : ", joint_positions)
                    
                    # apply action to FR3 

                    # arm joint 
                    
                    self.pub.publish(Float64MultiArray(data=joint_positions.tolist()))
                    
                    # gripper 

                    
                    if not self.gripper_grasp_position:

                        if gripper_action > 0.2: # 0.9
                            print("[INFO] 들어왔다  if 문에 ")
                            self.grasp_cli.send_goal(GraspGoal(width=CLOSE_GRIPPER_POSE, epsilon=GraspEpsilon(inner=0.01, outer=0.01),
                                                               speed=0.1, force=3.0))
                        else:
                            print("[INFO] 들어왔다 else 문에 ")

                            self.move_cli.send_goal(MoveGoal(width=OPEN_GRIPPER_POSE, speed=0.1))
                    
                    t+=1

                elif key == 'q': 
                    print("[EXIT] Simulation finished.")
                    break

            df = pd.DataFrame(action_list, columns = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "gripper"])
            now = datetime.now()
            timestamp = now.strftime("%y%m%d_%H%M")
            df.to_csv(f'actions_{timestamp}.csv', index=False)

if __name__ == "__main__":

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
    rospy.sleep(1)

    simulator.simulate()
