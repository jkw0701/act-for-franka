import sys
import os
import cv2
import cv_bridge
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
import h5py
import numpy as np

OPEN_GRIPPER_POSE = 0.08
GRASP = 0.041
DATASET_DIR = "/home/namyoon/ACTfranka/real_dir3/processed"

class CameraController:
    def __init__(self):
        rospy.init_node('camera_controller_node', anonymous=True)
        
        # 초기화
        self.current_joint_angles = [0.0] * 7
        self.current_gripper_width = OPEN_GRIPPER_POSE

        self.joint_state_sub = rospy.Subscriber('/franka_state_controller/joint_states', JointState, self.joint_state_callback)
        self.gripper_state_sub = rospy.Subscriber('/gripper/state', JointState, self.gripper_state_callback)

        self.bridge = cv_bridge.CvBridge()
        self.subscriber_left = rospy.Subscriber('/multisense_sl/camera/left/image_raw', Image, self.image_callback, callback_args='left')
        self.subscriber_right = rospy.Subscriber('/multisense_sl/camera/right/image_raw', Image, self.image_callback, callback_args='right')
        self.subscriber_gripper = rospy.Subscriber('/gripper_camera/image_raw', Image, self.image_callback, callback_args='gripper')

        # 초기값 설정
        self.data_left = []
        self.data_right = []
        self.data_gripper = []
        self.data_left2 = []
        self.data_right2 = []
        self.data_gripper2 = []
        self.joint_positions = []
        self.actions = []
        self.recording = False
        self.frame_count = 0
        self.joint_received = False
        self.gripper_received = False
        
        # 주기 설정 (예: 10Hz)
        self.rate = rospy.Rate(10)  # 10Hz
        
    def joint_state_callback(self, msg):
        self.current_joint_angles = list(msg.position[:7])

    def gripper_state_callback(self, msg):
        self.current_gripper_width = msg.position[0]

    def image_callback(self, msg, camera):
        if self.recording:
            # 각 카메라 이미지 처리
            if camera == 'left':
                self.current_frame_left = self.bridge.imgmsg_to_cv2(msg)
                self.data_left2.append(self.current_frame_left)
            elif camera == 'right':
                self.current_frame_right = self.bridge.imgmsg_to_cv2(msg)
                self.data_right2.append(self.current_frame_right)
            elif camera == 'gripper':
                self.current_frame_gripper = self.bridge.imgmsg_to_cv2(msg)
                self.data_gripper2.append(self.current_frame_gripper)

    def process_data(self):
        if self.data_left2 or self.data_right2 or self.data_gripper2:
            # 주기적으로 데이터를 저장
            joint_angles = list(self.current_joint_angles)
            gripper_status = 1 if self.current_gripper_width <= GRASP else 0
            joint_angles.append(gripper_status)
            self.joint_positions.append(joint_angles)
            self.data_left.append(self.data_left2[-1])
            self.data_right.append(self.data_right2[-1])
            self.data_gripper.append(self.data_gripper2[-1])

    def save_data(self):
        if not self.data_left or not self.data_right or not self.data_gripper:
            print(f"No data to save, skipping... this episode.")
            return

        episode_idx = 0
        directory = DATASET_DIR
        if not os.path.exists(directory):
            os.makedirs(directory)
        while os.path.exists(os.path.join(directory, f'episode_{episode_idx}.hdf5')):
            episode_idx += 1
        file_path = os.path.join(directory, f'episode_{episode_idx}.hdf5')
        
        with h5py.File(file_path, 'w') as root:
            root.attrs['sim'] = True
            obs = root.create_group('observations')
            images = obs.create_group('images')
            camera_names = ['left', 'right', 'gripper']
            for cam_name, data in zip(camera_names, [self.data_left, self.data_right, self.data_gripper]):
                image_data = np.array(data, dtype='uint8')
                print(image_data.shape)
                images.create_dataset(cam_name, data=image_data, dtype='uint8', chunks=(1, 480, 640, 3))
            qpos = np.array(self.joint_positions, dtype='float64')
            obs.create_dataset('qpos', data=qpos)
            action_data = np.array(self.joint_positions, dtype='float64')
            root.create_dataset('action', data=action_data)
        
        print(f"Data saved to {file_path}.")

    def reset_data(self):
        self.data_left = []
        self.data_right = []
        self.data_gripper = []
        self.data_left2 = []
        self.data_right2 = []
        self.data_gripper2 = []
        self.joint_positions = []
        self.frame_count = 0

    def capture_frames(self):
        print("Press 's' to start/stop recording. Press 'q' to quit.")
        
        cv2.namedWindow("Left Camera")
        cv2.namedWindow("Right Camera")
        cv2.namedWindow("Gripper Camera")

        while not rospy.is_shutdown():
            # 주기적인 작업 처리 (예: 이미지 화면 갱신)
            if self.data_left2:
                cv2.imshow("Left Camera", self.convert_image(self.data_left2[-1]))
            if self.data_right2:
                cv2.imshow("Right Camera", self.convert_image(self.data_right2[-1]))
            if self.data_gripper2:
                cv2.imshow("Gripper Camera", self.convert_image(self.data_gripper2[-1]))

            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):
                self.recording = not self.recording
                if self.recording:
                    print("Recording started.")
                    self.reset_data()  # 새로 시작할 때 데이터 초기화
                else:
                    print("[STOP] Recording stopped.")
                    print("Do you want to save the recorded data? (y/n)")
                    while True:
                        confirm_key = cv2.waitKey(0) & 0xFF
                        if confirm_key == ord('y'):
                            print("Saving data...")
                            self.save_data()
                            break
                        elif confirm_key == ord('n'):
                            print("Data discarded.")
                            break
                    print("Press 's' to start/stop recording. Press 'q' to quit.")
            elif key == ord('q'):
                break

            # 주기적으로 데이터를 처리
            self.process_data()
            self.rate.sleep()  # 주기 설정에 맞춰 대기

        cv2.destroyAllWindows()

    def convert_image(self, cv_image):
        height, width, channel = cv_image.shape
        bytesPerLine = 3 * width
        return cv_image


def main():
    camera_controller = CameraController()
    camera_controller.capture_frames()

if __name__ == '__main__':
    main()