import rospy
import numpy as np
import random
import cv2
import cv_bridge
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import h5py
from sensor_msgs.msg import Image
from controller.robot_state import *
from panda_kinematics import PandaWithPumpKinematics
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from std_srvs.srv import Empty
from settings.var import *
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from threading import Lock
from rich.progress import Progress
import tqdm
import time


class CameraController:
    def __init__(self, franka):
        self.bridge = cv_bridge.CvBridge()
        self.subscriber_left = rospy.Subscriber('/multisense_sl/camera/left/image_raw', Image, self.image_callback, callback_args='left')
        self.subscriber_right = rospy.Subscriber('/multisense_sl/camera/right/image_raw', Image, self.image_callback, callback_args='right')
        self.subscriber_gripper = rospy.Subscriber('/gripper_camera/image_raw', Image, self.image_callback, callback_args='gripper')

        self.current_frame_left = None
        self.current_frame_right = None
        self.current_frame_gripper = None

        self.data_left = []
        self.data_right = []
        self.data_gripper = []
        self.joint_positions = []
        self.actions = []
        self.franka = franka
        self.recording = False
        self.gripper_width = OPEN_GRIPPER_POSE
        self.box_positions = []
        self.frame_count = 0

    def image_callback(self, msg, camera):
        if self.recording:

            if camera == 'left':
                self.current_frame_left = self.bridge.imgmsg_to_cv2(msg)
                self.data_left.append(self.current_frame_left)
                # print(f"Added top frame {len(self.data_top)}")
            elif camera == 'right':
                self.current_frame_right = self.bridge.imgmsg_to_cv2(msg)
                self.data_right.append(self.current_frame_right)
                # print(f"Added top frame {len(self.data_top)}")
            elif camera == 'gripper':
                self.current_frame_gripper = self.bridge.imgmsg_to_cv2(msg)
                self.data_gripper.append(self.current_frame_gripper)
                # print(f"Added front frame {len(self.data_front)}")

                joint_angles = list(self.franka.angles())
                
                if self.franka.gripper_state() > 0.041 :
                    gripper_status = 0 
                
                elif self.franka.gripper_state() > 0.03 :
                    gripper_status = 1

                else: # 못잡았을 때 값을 지정할 수 있다.
                    gripper_status = 1
                
                joint_angles.append(gripper_status)
                self.joint_positions.append(joint_angles)

                print(self.franka.gripper_state(),gripper_status )



    def save_data(self):

        if not self.data_left or not self.data_right or not self.data_gripper: # or not self.data_front:
            print(f"No data to save, skipping... this episode. Left data: {len(self.data_left)}, Right data: {len(self.data_right)}, Gripper data: {len(self.data_gripper)}")
            return

        episode_idx = 0
        directory = DATASET_DIR


        if not os.path.exists(directory):
            os.makedirs(directory)
            print("[INFO] Directory created:", directory)
        
        while os.path.exists(os.path.join(directory, f'episode_{episode_idx}.hdf5')):
            episode_idx += 1

        file_path = os.path.join(directory, f'episode_{episode_idx}.hdf5')
        with h5py.File(file_path, 'w') as root:
            root.attrs['sim'] = True
            obs = root.create_group('observations')
            images = obs.create_group('images')
            camera_names = ['left', 'right','gripper']
            for cam_name, data in zip(camera_names, [self.data_left, self.data_right, self.data_gripper]):
                image_data = np.array(data, dtype='uint8')
                images.create_dataset(cam_name, data=image_data, dtype='uint8', chunks=(1, 480, 640, 3))
            qpos = np.array(self.joint_positions, dtype='float64')
            obs.create_dataset('qpos', data=qpos)
            box_positions = np.array(self.box_positions, dtype='float64')
            obs.create_dataset('box_positions', data=box_positions)
            action_data = np.array(self.joint_positions, dtype='float64')
            root.create_dataset('action', data=action_data)
        #     print("="*50)
        #     print(f"Top camera image data shape: {np.array(self.data_top).shape}")
        #     print(f"Front camera image data shape: {np.array(self.data_front).shape}")
        #     print(f"Joint positions shape: {qpos.shape}")
        #     print(f"Box positions shape: {box_positions.shape}")
        #     print(f"Action data shape: {action_data.shape}")
        print(f"Data saved to {file_path}.")
        # print("="*50)

    def start_recording(self):
        self.recording = True
        self.data_left = []
        self.data_right = []
        self.data_gripper = []
        self.joint_positions = []
        self.actions = []
        self.box_positions = []

    def stop_recording(self, save=True):
        self.recording = False
        if save:
            self.save_data()

    def log_box_position(self, x, y, z):
        if self.recording:
            self.box_positions.append([x, y, z])

    def log_failed_box_positions(self, x, y, z):
        with open('failed_positions.txt', 'a') as file:
            file.write(f'Failed Position - X: {x}, Y: {y}, Z: {z}\n')

class RobotTask:
    def __init__(self, camera_controller, franka):
        self.franka = RobotController()
        self.camera_controller = camera_controller
        self.kinematics = PandaWithPumpKinematics()
        self.initial_positions = INITIAL_JOINTS
        self.success_count = 0
        self.new_x = 0.5
        self.new_y = -0.2


        self.franka.move_to_joint_position_with_spline(self.initial_positions)

        self.set_box_position(self.new_x, self.new_y, BOX_Z)
        self.success_threshold = OPEN_GRIPPER_POSE
        self.endpoint = np.array([0.44, 0.2, 0.05])
        self.operate_gripper(OPEN_GRIPPER_POSE, GRIPPER_FORCE)  
        self.gripper_width = OPEN_GRIPPER_POSE  
        self.ori = np.array([np.pi, np.pi/2, 0.0, 0.0])


    def reset_episode(self):
        self.camera_controller.stop_recording(save=False)
        self.franka.move_to_joint_position_with_spline(self.initial_positions)
        if self.success_count < TOTAL_EPISODES: # type: ignore
            self.perform_task()

    def perform_task(self):
        with Progress() as progress:
            task_id = progress.add_task("[green]Generating episodes...", total=TOTAL_EPISODES)

            while self.success_count < TOTAL_EPISODES:

                self.franka.initial_pose() # move robot to initial pose 

                self.operate_gripper(OPEN_GRIPPER_POSE, GRIPPER_FORCE) # initialize gripper by opening it

                self.camera_controller.start_recording() # start recording
                self.camera_controller.log_box_position(self.new_x, self.new_y, BOX_Z) # refernce to save the box position not necessary 

                pre_pre_pick_pos, pre_pick_pos, pick_pos = self.update_box_pos2(self.new_x, self.new_y) 
                
                current_joint_angles = np.array(list(self.franka.angles()), dtype=np.float64)
                solution = self.solve_kinematics(current_joint_angles, pre_pre_pick_pos, self.ori) # solve kinematics and get positions to publish
                self.franka.move_to_joint_position_with_spline(solution.tolist(),duration=4.0)

                current_joint_angles = np.array(list(self.franka.angles()), dtype=np.float64)
                solution = self.solve_kinematics(current_joint_angles, pre_pick_pos, self.ori) # solve kinematics and get positions to publish
                self.move(solution)
                self.operate_gripper(OPEN_GRIPPER_POSE, GRIPPER_FORCE)
                
                if solution is None:
                    self.reset_episode()
                    continue
                
                if solution is None:
                    self.reset_episode()
                    continue
                
                ori = self.ori
                solution = self.solve_kinematics(current_joint_angles, pick_pos, ori)
                self.move(solution)
                time.sleep(0.1)
                self.camera_controller.gripper_width = GRASP
                self.franka.grasp(0.024, 15)
                self.move_up(pick_pos)
                self.place_pose()
                self.franka.move_to_joint_position_with_spline(self.initial_positions, duration=3.0, steps=750)

                self.camera_controller.stop_recording(save=False)

                if self.check_success():
                    print("[INFO] check success ")
                    self.camera_controller.save_data()
                    self.success_count += 1
                    progress.update(task_id, description=f"[green]Generating episodes... [white](Success: {self.success_count})", advance=1)
                else:
                    self.camera_controller.log_failed_box_positions(self.new_x, self.new_y, BOX_Z)
                    print("\033[91mEpisode Failed\033[0m")

                self.new_x, self.new_y = self.generate_cordinate()
                self.set_box_position(self.new_x, self.new_y, BOX_Z)

    def set_box_position(self, x, y, z):
        rospy.wait_for_service('/gazebo/set_model_state')
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        state_msg = SetModelStateRequest()
        state_msg.model_state.model_name = 'stone'
        state_msg.model_state.pose.position.x = x
        state_msg.model_state.pose.position.y = y
        state_msg.model_state.pose.position.z = z
        set_state(state_msg)

    def generate_cordinate(self):
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
        
        x = random.uniform(min_x, max_x)
        y = random.uniform(min_y, max_y)

        return x , y

    def set_box_position_from_input(self):
        print("Enter new box coordinates as x, y (e.g., -0.5, 0.5):")
        coords = input()
        try:
            x, y = map(float, coords.split(','))
            self.new_x, self.new_y = x, y
            self.set_box_position(x, y, BOX_Z)
        except ValueError:
            print("Invalid input. Using default position (-0.5, 0.5).")
            # self.new_x, self.new_y = 0, -0.2
            self.set_box_position(self.new_x, self.new_y, BOX_Z)

    def update_box_pos(self, new_x, new_y):
        z = BOX_Z
        pre_pre_pick_pos = np.array([new_x, new_y - 0.23 , z + 0.23], dtype=np.float64)
        pre_pick_pos = np.array([new_x, new_y - 0.01 , z + 0.1], dtype=np.float64)
        pick_pos = np.array([new_x + 0.006, new_y - 0.002, z - 0.0135], dtype=np.float64)
        return pre_pre_pick_pos, pre_pick_pos, pick_pos
    
    def update_box_pos2(self, new_x, new_y):
        z = BOX_Z
        pre_pre_pick_pos = np.array([new_x, new_y - 0.23 , z + 0.23], dtype=np.float64)
        pre_pick_pos = np.array([new_x, new_y - 0.1 , z + 0.1], dtype=np.float64)
        pick_pos = np.array([new_x, new_y - 0.0033, z - 0.0165], dtype=np.float64)
        return pre_pre_pick_pos, pre_pick_pos, pick_pos    
    
    def move_up(self , pick_pose):

        joint_positions = np.zeros(7)
        # print(pick_pose.tolist())
        self.franka.grasp(0.024 , 15) #그리퍼 안 놔
        pick_pose[2] += 0.1
        pick_pose[1] += 0.0

        solution = self.solve_kinematics(joint_positions ,pick_pose , self.ori)
        self.move(solution)


    def solve_kinematics(self, joint_positions, position, quat):
        return self.kinematics.ik(joint_positions, position, quat)

    def operate_gripper(self, width, f):
        self.gripper_width = width  # Update the local gripper width
        self.camera_controller.gripper_width = width  
        self.franka.exec_gripper_cmd(width, f)
        

    def move(self, solution):
        if solution is not None:
            self.franka.move_to_joint_position_with_spline(solution.tolist())


    def check_success(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        state_req = GetModelStateRequest(model_name='stone')
        state_res = get_state(state_req)
        box_pos = np.array([state_res.pose.position.x, state_res.pose.position.y, state_res.pose.position.z])

        print("[INFO] np.linalg.norm(self.endpoint - box_pos) : ", np.linalg.norm(self.endpoint - box_pos))

        return np.linalg.norm(self.endpoint - box_pos) < self.success_threshold

    def place_pose(self):
        self.franka.grasp(0.024 , 15) #그리퍼 안 놔
        joint_positions = np.zeros(7)
        pos = np.array([0.44, 0.21, 0.13])
        solution = self.solve_kinematics(joint_positions, pos, self.ori)
        self.move(solution)
        self.franka.grasp(0.024 , 15) #그리퍼 안 놔

        pos2 = np.array([0.44, 0.21, 0.05])

        solution2 = self.solve_kinematics(joint_positions, pos2, self.ori)

        if solution2 is not None:
            self.franka.move_to_joint_position_with_spline(solution2.tolist(),duration=1.0, steps=250)
    
        self.franka.exec_gripper_cmd(OPEN_GRIPPER_POSE ,0.5)
        self.move(solution)

def main():
    franka = RobotController()
    camera_controller = CameraController(franka)
    robot_task = RobotTask(camera_controller, franka)
    robot_task.perform_task()

if __name__ == '__main__':
    main()