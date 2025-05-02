# For Act policy
import os
import torch
import numpy as np
import pandas as pd 
import random
import pickle
import time 

from sensor_msgs.msg import Image
import rospy
from cv_bridge import CvBridge
from datetime import datetime
from sensor_msgs.msg import JointState
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))

from settings.var import POLICY_CONFIG, TASK_CONFIG, TRAIN_CONFIG
from rich.progress import Progress
from training.utils import make_policy, get_image

import readchar 
from datetime import datetime 
import actionlib
from franka_gripper.msg import MoveAction, MoveGoal, GraspAction, GraspGoal, GraspEpsilon   

# queue 

from queue import Queue
import threading 


DATASET_DIR = "/home/summit-nuc/Real_ACTFranka/real_dir3/processed"

OPEN_GRIPPER_POSE = 0.08 
CLOSE_GRIPPER_POSE = 0.025 

class Simulator:
    def __init__(self, policy, policy_config, task_config, stats, device):
        
        self.cfg = TASK_CONFIG
        rospy.init_node('csv_continuous_publisher')

        self.bridge = CvBridge() 

        self.image_cache = {}

        # for cn in self.cfg['camera_names']:
        #     topic = { 
        #         'left': '/camera1/color/image_raw',
        #         'right': '/camera2/color/image_raw',
        #         'gripper': '/camera3/color/image_raw'
        #     }.get(cn, '/camera1/color/image_raw')
            
        #     rospy.Subscriber(topic, Image, lambda msg, cn=cn:self.image_callback(msg,cn))

            
        # ===================== 3층 내려가서 home pose로 바꿔주기.. =========================
        self.current_joint_angles = [0.0] * 7 
        # ===================== 3층 내려가서 home pose로 바꿔주기.. =========================
        self.current_gripper_width = OPEN_GRIPPER_POSE 
        self.joint_names = rospy.get_param('~joint_names', [
            'panda_joint1','panda_joint2','panda_joint3','panda_joint4',
            'panda_joint5','panda_joint6','panda_joint7'
        ])

        self.publish_hz = rospy.get_param('~publish_hz', 50.0)
        self.rate = rospy.Rate(self.publish_hz) # 50Hz 

        self.joint_state_sub = rospy.Subscriber('/franka_state_controller/joint_states', JointState, self.joint_state_callback)
        self.gripper_state_sub = rospy.Subscriber('/franka_gripper/joint_states', JointState, self.gripper_state_callback)

        # For Arm joint 
        self.target_topic = rospy.get_param('~target_topic', '/target_joint_states')
        self.pub = rospy.Publisher(self.target_topic, JointState, queue_size=1)

        # For Gripper 
        self.move_cli = actionlib.SimpleActionClient("/franka_gripper/move", MoveAction)
        self.grasp_cli = actionlib.SimpleActionClient("/franka_gripper/grasp", GraspAction)
        
        rospy.loginfo("Waiting for gripper servers…"); self.move_cli.wait_for_server(); self.grasp_cli.wait_for_server()
        self.move_cli.wait_for_server()
        self.grasp_cli.wait_for_server()


        self.policy = policy
        self.policy_config = policy_config
        self.cfg = task_config
        self.device = device
        self.stats = stats
        self.pre_process = lambda pos: (pos - stats['qpos_mean']) / stats['qpos_std']
        self.post_process = lambda act: act * stats['action_std'] + stats['action_mean']
        self.gripper_grasp_position = False

        self.joint_positions = []
        self.joint_received = False
        self.gripper_received = False

        self.policy_input_queue = Queue()
        self.policy_output_queue = Queue()
        threading.Thread(target=self.policy_worker, daemon=True).start()


    def image_callback(self, msg, camera_name):
        try:
            self.image_cache[camera_name] = self.bridge.imgmsg_to_cv2(msg,"bgr8")

        except Exception as e : 
            rospy.logwarn(f"[Warn] Error converting image : {e}")

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
        # return self.image_cache.get(camera_name, np.zeros((480,640,3), dtype=np.uint8))
    
    def policy_worker(self):
        while not rospy.is_shutdown():
            t_input, qpos_input, image = self.policy_input_queue.get()
            with torch.no_grad():
                actions = self.policy(qpos_input, image)
            self.policy_output_queue.put((t_input, actions))

    def simulate(self):

        query_frequency = self.policy_config['num_queries'] #149 
        
        if self.policy_config['temporal_agg']:
            query_frequency = 30 # 20
            num_queries = self.policy_config['num_queries']
            all_time_actions = torch.zeros([self.cfg['episode_len'], self.cfg['episode_len']+num_queries, self.cfg['state_dim']]).to(self.device)

        qpos_history = torch.zeros((1, self.cfg['episode_len'], self.cfg['state_dim'])).to(self.device)

        with Progress() as progress:

            # hz = 10
            # rate = rospy.Rate(hz)  # 30Hz 루프
            
            task = progress.add_task("[green]Running real-time simulation...", total=self.cfg['episode_len'])
            t = 0 

            for t in range(self.cfg['episode_len']):

                print(f"[Progress] {t} / {self.cfg['episode_len']} Steps")
                
                progress.update(task, advance=1)
    
                qpos = self.current_joint_angles
                gripper_width = self.current_gripper_width 
                gripper_binary = 0 if gripper_width > 0.04 else 1
                pos = np.append(qpos, gripper_binary)

                images = {cn: self.capture_image(cn) for cn in self.cfg['camera_names']}
                qpos_input = torch.from_numpy(self.pre_process(pos)).float().to(self.device).unsqueeze(0)
                qpos_history[:, t] = qpos_input
                curr_image = get_image(images, self.cfg['camera_names'], self.device)

                if t % query_frequency == 0:
                    print("[INFO] Running policy")
                    self.policy_input_queue.put((t, qpos_input, curr_image))

                while not self.policy_output_queue.empty():
                    t_result, all_actions = self.policy_output_queue.get()
                    all_time_actions[[t_result], t_result:t_result+num_queries] = all_actions

                if t == 0 :
                    time.sleep(0.5)
                    t_result, all_actions = self.policy_output_queue.get()
                    all_time_actions[[t_result], t_result:t_result+num_queries] = all_actions

                    
                if self.policy_config['temporal_agg']:
                    all_time_actions[[t], t:t+num_queries] = all_actions
                    actions_for_curr_step = all_time_actions[:, t+10] # t
                    actions_populated = torch.all(actions_for_curr_step != 0, axis=1) # True or False 
                    actions_for_curr_step = actions_for_curr_step[actions_populated]
                    
                    k = 0.01 # t = k:0.01
                    exp_weights = np.exp(-k * np.arange(len(actions_for_curr_step)))
                    exp_weights = exp_weights / exp_weights.sum()
                    exp_weights = torch.from_numpy(exp_weights.astype(np.float32)).to(self.device).unsqueeze(dim=1)
                    raw_action = (actions_for_curr_step * exp_weights).sum(dim=0, keepdim=True)

                else:
                    raw_action = all_actions[:, t % query_frequency]

                action = self.post_process(raw_action.squeeze(0).detach().cpu().numpy())
            
                joint_positions = action[:7]
                gripper_action = action[7]

                print("[INFO] grippper action : ", gripper_action)
                
                # apply action to FR3 
                
                js = JointState()
                js.header.stamp = rospy.Time.now()
                js.name = self.joint_names
                js.position = joint_positions 

                self.pub.publish(js) # arm 
                
                # gripper 
                
                if not self.gripper_grasp_position:

                    if gripper_action > 0.5: # 0.9
                        print("[INFO] !!!!!!!!!!!!!!!!잡았다!!!!!!!!!!!!!!!")
                        self.grasp_cli.send_goal(GraspGoal(width=CLOSE_GRIPPER_POSE, epsilon=GraspEpsilon(inner=0.013, outer=0.013),
                                                            speed=0.1, force=15.0))
                        self.gripper_grasp_position = True
                    
                if self.gripper_grasp_position:

                    if gripper_action < 0.5:
                        print("[INFO] !!!!!!!!!!!!!!!놓았다!!!!!!!!!!!!!!!")
                        self.move_cli.send_goal(MoveGoal(width=OPEN_GRIPPER_POSE, speed=0.1))
                        self.gripper_grasp_position = False
                
                # if gripper_action > 0.5: # 0.9
                #     print("[INFO] !!!!!!!!!!!!!!!!잡았다!!!!!!!!!!!!!!!")
                #     self.grasp_cli.send_goal(GraspGoal(width=CLOSE_GRIPPER_POSE, epsilon=GraspEpsilon(inner=0.013, outer=0.013),
                #                                         speed=0.1, force=15.0))
                # else:
                #     print("[INFO] !!!!!!!!!!!!!!!놓았다!!!!!!!!!!!!!!!")
                #     self.move_cli.send_goal(MoveGoal(width=OPEN_GRIPPER_POSE, speed=0.1))
                #     # self.gripper_grasp_position = False


                # rate.sleep()


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
