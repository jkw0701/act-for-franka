import torch
import gc
import numpy as np
import pandas as pd 
import random
import rospy
import pickle
import csv
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from franka_core_msgs.msg import JointCommand
from sensor_msgs.msg import JointState
from franka_gripper.msg import MoveActionGoal, GraspAction, GraspActionGoal, GraspGoal, GraspEpsilon

import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))

from panda_kinematics import PandaWithPumpKinematics
from settings.var import GRIPPER_FORCE, BOX_Z, POLICY_CONFIG, TASK_CONFIG, TRAIN_CONFIG
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from rich.progress import Progress
from training.utils import make_policy, get_image


class PIDController:
    def __init__(self, p_gain = 0.05, i_gain = 0.0, d_gain = 0.01 , dt=0.001):
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
        joint_state_topic='/joint_states'
        controller_name='/panda_simulator/motion_controller/arm/joint_commands'
        self.joints = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
        self.gripper_positions = []
        self.current_positions = [None] * 7

        #입력값 : 조인트, 그리퍼 상태 / 출력값 : 조인트 액션 ++ 이미지 입력은 아래 다른 방식으로 한다.
        self.joint_state_subscriber = rospy.Subscriber(joint_state_topic, JointState, self.joint_state_callback)
        self.gripper_state_subscriber = rospy.Subscriber('/franka_gripper/joint_states', JointState, self.gripper_joint_state_callback)      
        self.trajectory_publisher = rospy.Publisher(controller_name, JointCommand, queue_size=10)
        self.gripper_publisher = rospy.Publisher('/franka_gripper/move/goal', MoveActionGoal, queue_size=10)
        self.grasp_publisher = rospy.Publisher('/franka_gripper/grasp/goal', GraspActionGoal, queue_size=10)

        self.policy = policy
        self.policy_config = policy_config
        self.cfg = task_config
        self.device = device
        self.stats = stats
        self.pre_process = lambda pos: (pos - stats['qpos_mean']) / stats['qpos_std']
        self.post_process = lambda act: act * stats['action_std'] + stats['action_mean']
        self.bridge = CvBridge()

        rospy.sleep(0.1)
        print("현재 포지션 입력 대기")

        self.initial_pose()     
        self.endpoint = np.array([0.44, 0.2, 0.05])   
        self.success_threshold = 0.1
        self.gripper_grasp_position = False

    def gripper_joint_state_callback(self, msg):
        positions = msg.position
        if positions is not None:
            self.gripper_positions = positions

    def gripper_state(self):
        if not self.gripper_positions:
            rospy.logwarn("No gripper positions received yet.")
            return None
        
        position = sum(self.gripper_positions)

        #rospy.loginfo(f"gripper distance: {position}")
        return position

    def joint_state_callback(self, msg):
        positions = [None] * len(self.joints)
        for idx, name in enumerate(self.joints):
            if name in msg.name:
                pos_idx = msg.name.index(name)
                positions[idx] = msg.position[pos_idx]
        self.current_positions = positions

    def angles(self):
        if self.current_positions is None:
            rospy.logwarn("Current positions have not been received yet.")
            return None
        return self.current_positions
    
    def exec_gripper_cmd(self ,width, speed=0.5):
        # width = 0.08, speed = 1 
        while self.gripper_publisher.get_num_connections() == 0:
            rospy.sleep(0.1)
        move_goal = MoveActionGoal()

        move_goal.goal.width = width
        move_goal.goal.speed = speed

        self.gripper_publisher.publish(move_goal)

    def grasp(self,width,force=5,speed=0.1,inner_epsilon=0.005,outer_epsilon=0.005):
               
        if not rospy.core.is_initialized():
            rospy.init_node('gripper_command_publisher', anonymous=True)

     
        grasp_goal = GraspGoal()
        grasp_goal.width = width
        grasp_goal.speed = speed
        grasp_goal.force = force
        grasp_goal.epsilon.inner = inner_epsilon
        grasp_goal.epsilon.outer = outer_epsilon

      
        action_goal = GraspActionGoal()
        action_goal.goal = grasp_goal

        # Publish the message
        self.grasp_publisher.publish(action_goal)
        # rospy.loginfo("Published grasp goal to /franka_gripper/grasp/goal")

    def initial_pose(self):

        angles = [0, -0.7, 0, -2.35619449, 0, 1.57079632679, 0.785398163397]
        self.move_to_joint_position_with_PID(angles ,duration=2)

    ## 시뮬 상 박스 위치를 부여하고 성공 여부를 판단하는 함수들// 실제 동작에서는 제거해도 된다.
    def check_success(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        state_req = GetModelStateRequest(model_name='stone')
        state_res = get_state(state_req)
        box_pos = np.array([state_res.pose.position.x, state_res.pose.position.y, state_res.pose.position.z])

        print("[INFO] np.linalg.norm(self.endpoint - box_pos) : ", np.linalg.norm(self.endpoint - box_pos))

        return np.linalg.norm(self.endpoint - box_pos) < self.success_threshold

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
            box_length = 0.05
            box_width = 0.05
            # Position of the larger box (center coordinates)
            box_x_center = 0.45
            box_y_center = -0.21
            cube_x = 0.015
            cube_y = 0.032
            min_x = box_x_center - box_length / 2 + cube_x / 2
            max_x = box_x_center + box_length / 2 - cube_x / 2
            min_y = box_y_center - box_width / 2 + cube_y / 2
            max_y = box_y_center + box_width / 2 - cube_y / 2
            
            x = random.uniform(min_x , max_x)
            y = random.uniform(min_y, max_y)
            return x , y

    def move_to_joint_position_with_PID(self, target_angles, duration=0.5, p_gain = 0.055, i_gain = 0.0, d_gain = 0.0005 ):
        if self.current_positions is None:
            rospy.logwarn("Current joint positions not available.")
            return
        dt = 0.001
        steps = int(duration/dt)
        end = np.array(target_angles)
        times = np.linspace(0, duration, steps)
        
        hz = 1 / dt
        pid_controllers = [PIDController(p_gain = p_gain, i_gain = i_gain, d_gain = d_gain , dt=dt) for _ in range(7)]
        rate = rospy.Rate(hz)

        for i, t in enumerate(times):
            position = []

            for j in range(7):
                # PID 제어를 이용한 위치 계산
                pid_output = pid_controllers[j].compute(end[j], self.current_positions[j])
                new_position = self.current_positions[j] + pid_output
                position.append(new_position)

            # ROS 메시지 생성 및 발행
            msg = JointCommand()
            msg.header.stamp = rospy.Time.now()
            msg.mode = JointCommand.POSITION_MODE
            msg.names = self.joints
            msg.position = position
            #msg.velocity = velocity
            #msg.acceleration = acceleration
            self.trajectory_publisher.publish(msg)
            rate.sleep()


        rate.sleep()

        #rospy.loginfo("Finished sending PID-controlled trajectory.")

    # 여기서 이미지 입력값을 가져온다. 일반적인 입력과 다름
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
            for t in range(self.cfg['episode_len']):

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
                    duration = 0.1 * position_error
                    p_gain = 0.03
                    i_gain = 0.0
                    d_gain = 0.003
                    self.move_to_joint_position_with_PID(joint_positions, duration = duration, p_gain = p_gain, i_gain = i_gain, d_gain = d_gain)

                #print("gripper action: ", gripper_action)
                
                if not self.gripper_grasp_position:

                    if gripper_action > 0.2: # 0.9
                        print("IN gripper action !!!!!!!!! ")
                        self.grasp(0.0249, 5)
                        #self.gripper_grasp_position = True

                    else:
                        print("in else grippger aciton !!!!!!!!!! ")
                        self.exec_gripper_cmd(0.08, GRIPPER_FORCE)

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

    #성공하는 박스 위치의 분포를 알기위해 반복문을 넣음
    for a in range(30):
        simulator.initial_pose()  
        box_x, box_y = simulator.generate_coordinate()
        simulator.set_box_position(box_x, box_y, BOX_Z)
        rospy.sleep(1)
        simulator.simulate()

        if simulator.check_success():
            rospy.loginfo(f"성공한 박스 위치: x={box_x}, y={box_y}")
        else:
            rospy.loginfo(f"실패한 박스 위치: x={box_x}, y={box_y}")
