import sys
import os
# 현재 파일 기준으로 상위 폴더를 모듈 경로에 추가
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
import cv2
import cv_bridge
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
import os
import h5py
import numpy as np

OPEN_GRIPPER_POSE = 0.08
GRASP = 0.025
DATASET_DIR = "/home/namyoon/ACTfranka/real_dir3/processed"
# DATASET_DIR = "/home/summit-nuc/ACTFranka/real_dir3/processed"

#/multisense_sl/camera/left/image_raw 카메라 노드의 메세지를 기준으로 바뀔 수 있다.

class CameraController:
    def __init__(self):
        rospy.init_node('camera_controller_node', anonymous=True)
        # 추가: joint & gripper 상태 초기화
        self.current_joint_angles = [0.0] * 7
        self.current_gripper_width = OPEN_GRIPPER_POSE

        # 프랑카 상태 구독자 추가
        self.joint_state_sub = rospy.Subscriber('/franka_state_controller/joint_states',JointState,self.joint_state_callback)
        self.gripper_state_sub = rospy.Subscriber('/gripper/state',JointState,self.gripper_state_callback)  # 토픽 이름은 실제 상황에 맞게 수정
        # self.gripper_state_sub = rospy.Subscriber('/franka_gripper/joint_states',JointState,self.gripper_state_callback)

        self.bridge = cv_bridge.CvBridge()
        self.subscriber_left = rospy.Subscriber('/multisense_sl/camera/left/image_raw', Image, self.image_callback, callback_args='left')
        self.subscriber_right = rospy.Subscriber('/multisense_sl/camera/right/image_raw', Image, self.image_callback, callback_args='right')
        self.subscriber_gripper = rospy.Subscriber('/gripper_camera/image_raw', Image, self.image_callback, callback_args='gripper')

        # self.subscriber_left = rospy.Subscriber('/camera1/color/image_raw', Image, self.image_callback, callback_args='left')
        # self.subscriber_right = rospy.Subscriber('/camera2/color/image_raw', Image, self.image_callback, callback_args='right')
        # self.subscriber_gripper = rospy.Subscriber('/camera3/color/image_raw', Image, self.image_callback, callback_args='gripper')

        self.current_frame_left = None
        self.current_frame_right = None
        self.current_frame_gripper = None
        self.data_left = []
        self.data_right = []
        self.data_gripper = []
        self.joint_positions = []
        self.actions = []
        self.recording = False
        self.gripper_width = OPEN_GRIPPER_POSE
        self.box_positions = []
        self.frame_count = 0
        self.joint_received = False  #조인트 메세지 초기값 플래그
        self.gripper_received = False


    def joint_state_callback(self, msg):# JointState.position: 리스트로 7개 관절 각도 제공
        self.current_joint_angles = list(msg.position[:7])
        self.joint_received = True

    def gripper_state_callback(self, msg):
        self.current_joint_angles = msg.position[0]
        self.gripper_received = True

    def image_callback(self, msg, camera):
        if self.recording:
            if camera == 'left':
                self.current_frame_left = self.bridge.imgmsg_to_cv2(msg)
                # (msg, desired_encoding="bgr8")
                self.data_left.append(self.current_frame_left)
                # print(f"Added left frame {len(self.data_left)}")

            elif camera == 'right':
                self.current_frame_right = self.bridge.imgmsg_to_cv2(msg)
                self.data_right.append(self.current_frame_right)
                # print(f"Added right frame {len(self.data_right)}")
                
            elif camera == 'gripper':
                self.current_frame_gripper = self.bridge.imgmsg_to_cv2(msg)
                self.data_gripper.append(self.current_frame_gripper)
                # print(f"Added gripper frame {len(self.data_gripper)}")

                # ✅ ROS에서 받은 값을 사용하도록 수정
                if self.recording and self.joint_received and self.gripper_received:

                    joint_angles = list(self.current_joint_angles)
                    gripper_status = 1 if self.current_gripper_width <= GRASP else 0
                    joint_angles.append(gripper_status)
                    self.joint_positions.append(joint_angles)


    def save_data(self):
        if not self.data_left or not self.data_right or not self.data_gripper:
            print(f"No data to save, skipping... this episode. left data: {len(self.data_left)}, right data: {len(self.data_right)}, gripper data: {len(self.data_gripper)}")
            #return

        episode_idx = 0
        directory = DATASET_DIR

        if not os.path.exists(directory):
            os.makedirs(directory)
        while os.path.exists(os.path.join(directory, f'episode_{episode_idx}.hdf5')):
            episode_idx += 1
        file_path = os.path.join(directory, f'episode_{episode_idx}.hdf5')
        with h5py.File(file_path, 'w') as root:
            root.attrs['sim'] = True   ## 이거 바꿔야 함
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
        #     print("="*50)
        #     print(f"Top camera image data shape: {np.array(self.data_top).shape}")
        #     print(f"Front camera image data shape: {np.array(self.data_front).shape}")
        #     print(f"Joint positions shape: {qpos.shape}")
        #     print(f"Box positions shape: {box_positions.shape}")
        #     print(f"Action data shape: {action_data.shape}")
        print(f"Data saved to {file_path}.")
        # print("="*50)

    def convert_image(self, cv_image):
        height, width, channel = cv_image.shape
        bytesPerLine = 3 * width
        #return QImage(cv_image.data, width, height, bytesPerLine, QImage.Format_RGB888).rgbSwapped()
        return cv_image

    def capture_frames(self):
        print("Press 's' to start/stop recording. Press 'q' to quit.")
        
        cv2.namedWindow("Left Camera")
        cv2.namedWindow("Right Camera")
        cv2.namedWindow("Gripper Camera")

        while True:
            # OpenCV로 화면 띄우기 (프레임이 있을 때만)
            if self.data_left:
                cv2.imshow("Left Camera", self.convert_image(self.data_left[-1]))
            if self.data_right:
                cv2.imshow("Right Camera", self.convert_image(self.data_right[-1]))
            if self.data_gripper:
                cv2.imshow("Gripper Camera", self.convert_image(self.data_gripper[-1]))

            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):
                self.recording = not self.recording
                if self.recording:
                    print("Recording started.")
                    self.data_left = []
                    self.data_right = []
                    self.data_gripper = []
                    self.joint_positions = [] # Reset data lists for new recording
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
                        else:
                            print("Press 'y' to save or 'n' to discard.")

            elif key == ord('q'):
                break

        cv2.destroyAllWindows()

def main():
    camera_controller = CameraController()
    camera_controller.capture_frames()

if __name__ == '__main__':
    main()
