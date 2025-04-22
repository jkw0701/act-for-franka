import torch
import gc
import numpy as np
import pandas as pd 
import rospy
import pickle
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from franka_gripper.msg import MoveAction,  GraspAction, GraspGoal, GraspEpsilon

import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))

from settings.var import GRIPPER_FORCE, BOX_Z, POLICY_CONFIG, TASK_CONFIG, TRAIN_CONFIG
from rich.progress import Progress
from training.utils import make_policy, get_image
import readchar 
import actionlib

OPEN_GRIPPER_POSE = 0.08 
CLOSE_GRIPPER_POSE = 0.03 

class PIDController:
    def __init__(self, p_gain = 0.8, i_gain = 0.01, d_gain = 0.01 , dt=0.001):
        self.p_gain = p_gain
        self.i_gain = i_gain
        self.d_gain = d_gain
        self.dt = dt
        self.prev_error = 0
        self.integral = 0

    def compute(self, target, current):
        error = target - current
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        output = self.p_gain * error + self.i_gain * self.integral + self.d_gain * derivative
        self.prev_error = error
        return output

class Simulator:
    def __init__(self, policy, policy_config, task_config, stats, device):

        rospy.init_node('robot_controller', anonymous=True)
        #joint_state_topic='/joint_states'
        #controller_name='/panda_simulator/motion_controller/arm/joint_commands'
        self.joints = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
        self.gripper_positions = []
        self.current_positions = [None] * 7

        #입력값 : 조인트, 그리퍼 상태 / 출력값 : 조인트 액션 ++ 이미지 입력은 아래 다른 방식으로 한다.
        self.joint_state_sub = rospy.Subscriber('/franka_state_controller/joint_states', JointState, self.joint_state_callback)
        self.gripper_state_sub = rospy.Subscriber('/franka_gripper/joint_states', JointState, self.gripper_state_callback)
        self.pub = rospy.Publisher('/joint_group_position_controller/command',Float64MultiArray, queue_size=1)
        
        self.move_cli = actionlib.SimpleActionClient("/franka_gripper/move", MoveAction)
        self.grasp_cli = actionlib.SimpleActionClient("/franka_gripper/grasp", GraspAction)

        self.policy = policy
        self.policy_config = policy_config
        self.cfg = task_config
        self.device = device
        self.stats = stats
        self.pre_process = lambda pos: (pos - stats['qpos_mean']) / stats['qpos_std']
        self.post_process = lambda act: act * stats['action_std'] + stats['action_mean']
        self.bridge = CvBridge()

        while self.current_positions is None:
            rospy.sleep(0.1)
            print("노드 안켜짐")
        self.initial_pose(self.current_positions)     
        self.gripper_grasp_position = False


    def joint_state_callback(self, msg):
        self.current_positions = list(msg.position[:7])

    def gripper_state_callback(self, msg):
        self.gripper_positions = msg.position[0]


    def gripper_state(self):
        if not self.gripper_positions:
            rospy.logwarn("No gripper positions received yet.")
            return None
        
        position = sum(self.gripper_positions)

        #rospy.loginfo(f"gripper distance: {position}")
        return position

    def angles(self):
        if self.current_positions is None:
            rospy.logwarn("Current positions have not been received yet.")
            return None
        return self.current_positions

    def initial_pose(self, current_positions):

        angles = [0, -0.7, 0, -2.35619449, 0, 1.57079632679, 0.785398163397]
        self.move_to_joint_position_with_PID(current_positions , angles ,duration=5)

    def move_to_joint_position_with_PID(self, current_positions, target_angles, duration=0.5):
        if current_positions is None:
            rospy.logwarn("Current joint positions not available.")
            return
        dt = 0.0003
        steps = int(duration/dt)
        start = np.array(current_positions)
        end = np.array(target_angles)
        times = np.linspace(0, duration, steps)
        
        hz = 1 / dt
        pid_controllers = [PIDController() for _ in range(7)]
        rate = rospy.Rate(hz)
        
        # 이전 위치 및 속도 저장용 변수 초기화
        prev_positions = list(current_positions)
        prev_velocities = [0.0] * 7
        time_step = duration / steps

        for i, t in enumerate(times):
            position = []
            #velocity = []
            #acceleration = []

            for j in range(7):
                # PID 제어를 이용한 위치 계산
                pid_output = pid_controllers[j].compute(end[j], self.current_positions[j])
                new_position = current_positions[j] + pid_output
                position.append(new_position)

            self.pub.publish(Float64MultiArray(data=position.tolist()))
            rate.sleep()


        rate.sleep()

        #rospy.loginfo("Finished sending PID-controlled trajectory.")

    # 여기서 이미지 입력값을 가져온다. 일반적인 입력과 다름
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
            query_frequency = 5 # 30  
            num_queries = self.policy_config['num_queries']
            all_time_actions = torch.zeros([self.cfg['episode_len'], self.cfg['episode_len']+num_queries, self.cfg['state_dim']]).to(self.device)
        
        else:
            num_queries = self.policy_config['num_queries']
            all_time_actions = torch.zeros([self.cfg['episode_len'], self.cfg['episode_len']+3, self.cfg['state_dim']]).to(self.device)

        qpos_history = torch.zeros((1, self.cfg['episode_len'], self.cfg['state_dim'])).to(self.device)

        with Progress() as progress:
            task = progress.add_task("[green]Running real-time simulation...", total=self.cfg['episode_len'])
            action_list = [] #csv에 저장할 액션리스트
            t = 0 

            while t < self.cfg['episode_len']:
                key = readchar.readkey()
                print(f"[Progress] {t} / {self.cfg['episode_len']} Steps")
                
                if key == ' ': 

                    progress.update(task, advance=1)

                    qpos = self.angles()
                    gripper_width = self.gripper_state()
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
                            all_time_actions[[t], t:t+num_queries] = all_actions

                        
                    if self.policy_config['temporal_agg']:
                        all_time_actions[[t], t:t+num_queries] = all_actions
                        actions_for_curr_step = all_time_actions[:, t]
                        actions_populated = torch.all(actions_for_curr_step != 0, axis=1)
                        actions_for_curr_step = actions_for_curr_step[actions_populated]
                        k = 0.001
                        exp_weights = np.exp(-k * np.arange(len(actions_for_curr_step)))
                        exp_weights = exp_weights / exp_weights.sum()
                        exp_weights = torch.from_numpy(exp_weights.astype(np.float32)).to(self.device).unsqueeze(dim=1)
                        raw_action = (actions_for_curr_step * exp_weights).sum(dim=0, keepdim=True)
                    else:
                        raw_action = all_actions[:, t % query_frequency]

                    action = self.post_process(raw_action.squeeze(0).detach().cpu().numpy())
                    #action_list.append(action.tolist()) # Convert numpy array to list before appending 
                    # Apply action to robot
                    self.current_positions = self.angles()

                    joint_positions = action[:7]
                    #print(joint_positions)
                    gripper_action = action[7]

                    position_error = max(np.abs(np.array(joint_positions) - np.array(self.current_positions)))
                    #print("------------------가야하는 거리" , position_error)

                    # 허용 오차보다 크면 정지
                    if position_error > 0.8:  # threshold는 상황에 맞게 조정
                        #joint_positions = (self.current_positions + 0.1* joint_positions) / 1.1
                        joint_positions = self.current_positions
                        print(f"Position error too large: {position_error}")
                    else:
                        #if t % 10 == 0:
                        self.move_to_joint_position_with_PID(self.current_positions , joint_positions, duration = 3.0 * position_error)

                    #print("gripper action: ", gripper_action)
                    
                    if not self.gripper_grasp_position:

                        if gripper_action > 0.2: # 0.9
                            print("IN gripper action !!!!!!!!! ")
                            self.grasp_cli.send_goal(GraspGoal(width=CLOSE_GRIPPER_POSE, epsilon=GraspEpsilon(inner=0.01, outer=0.01),
                                                                speed=0.1, force=3.0))
                            #self.gripper_grasp_position = True

                        else:
                            print("in else grippger aciton !!!!!!!!!! ")
                            self.move_cli.send_goal(MoveGoal(width=OPEN_GRIPPER_POSE, speed=0.1))

                elif key == 'q': 
                        print("[EXIT] Simulation finished.")
                        break

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
    rospy.sleep(1)

    simulator.simulate()