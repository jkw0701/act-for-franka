import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from franka_gripper.msg import MoveActionGoal, GraspAction, GraspActionGoal, GraspGoal, GraspEpsilon
from scipy.interpolate import CubicSpline
import numpy as np
import actionlib

import sys
sys.path.append('~/ros/ws_ACTfranka/devel/lib/python3/dist-packages')

from franka_core_msgs.msg import JointCommand

class PIDController:
    def __init__(self, p_gain, i_gain, d_gain, dt=0.01):
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

class RobotController:
    def __init__(self, controller_name='/panda_simulator/motion_controller/arm/joint_commands', joint_state_topic='/joint_states'):
        # /position_joint_trajectory_controller/command
        rospy.init_node('robot_controller', anonymous=True)
        self.trajectory_publisher = rospy.Publisher(controller_name, JointCommand, queue_size=10)
        self.gripper_publisher = rospy.Publisher('/franka_gripper/move/goal', MoveActionGoal, queue_size=10)
        self.grasp_publisher = rospy.Publisher('/franka_gripper/grasp/goal', GraspActionGoal, queue_size=10)
        
        self.joint_state_subscriber = rospy.Subscriber(joint_state_topic, JointState, self.joint_state_callback)
        self.gripper_state_subscriber = rospy.Subscriber('/franka_gripper/joint_states', JointState, self.gripper_joint_state_callback)
        
        self.joints = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
        self.gripper_positions = []
        self.current_positions = [None] * 7
        self.position_tolerance = 0.001  # Tolerance for position errors in radians
        self.grasp_client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
        rospy.loginfo("Waiting for /franka_gripper/grasp action server...")
        self.grasp_client.wait_for_server()
        rospy.loginfo("Grasp action server ready.")


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


    def move_to_joint_position(self, angles, duration = 0.5):
        if len(angles) != 7:
            raise ValueError("Exactly 7 angles must be provided.")
        # rospy.loginfo("Moving robot to specified angles...")
        '''
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joints
        point = JointTrajectoryPoint()
        point.positions = angles
        point.velocities = [0.0] * 7
        point.accelerations = [0.0] * 7
        point.time_from_start = rospy.Duration(5)
        rospy.sleep(0.5)
        trajectory_msg.points.append(point)
        '''
        trajectory_msg = JointCommand()
        trajectory_msg.names = self.joints
        trajectory_msg.mode = 1
        trajectory_msg.header.stamp = rospy.Time.now()
        trajectory_msg.position = angles
        
        #print(angles)

        self.trajectory_publisher.publish(trajectory_msg)
        rospy.loginfo("Command published to move the robot.")

        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < duration:
            if self.current_positions is None:
                rospy.logwarn("Current positions have not been received yet.")
                rospy.sleep(0.1)
                continue

            if self.positions_close_enough(self.current_positions, angles):
                # rospy.loginfo("Target positions reached.")
                break
            rospy.sleep(0.1)
        # else:
        #     rospy.logwarn("Timeout reached before reaching the target positions.")


    def move_to_joint_position_with_PID(self, target_angles, duration=0.5):
        if self.current_positions is None:
            rospy.logwarn("Current joint positions not available.")
            return

        steps = int(duration/0.001)
        start = np.array(self.current_positions)
        end = np.array(target_angles)
        times = np.linspace(0, duration, steps)
        
        hz = steps / duration
        dt = 1.0 / hz
        pid_controllers = [PIDController(0.8, 0.01, 0.01, dt) for _ in range(7)]
        rate = rospy.Rate(hz)
        
        # 이전 위치 및 속도 저장용 변수 초기화
        prev_positions = list(self.current_positions)
        prev_velocities = [0.0] * 7
        time_step = duration / steps

        for i, t in enumerate(times):
            position = []
            velocity = []
            acceleration = []

            for j in range(7):
                # PID 제어를 이용한 위치 계산
                pid_output = pid_controllers[j].compute(end[j], self.current_positions[j])
                new_position = self.current_positions[j] + pid_output
                position.append(new_position)

                # 속도 = (현재 위치 - 이전 위치) / 시간 간격
                vel = (new_position - prev_positions[j]) / time_step
                velocity.append(vel)

                # 가속도 = (현재 속도 - 이전 속도) / 시간 간격
                acc = (vel - prev_velocities[j]) / time_step
                acceleration.append(acc)

                # 이전 상태 갱신
                prev_positions[j] = new_position
                prev_velocities[j] = vel

            # ROS 메시지 생성 및 발행
            msg = JointCommand()
            msg.header.stamp = rospy.Time.now()
            msg.mode = JointCommand.POSITION_MODE
            msg.names = self.joints
            msg.position = position
            msg.velocity = velocity
            msg.acceleration = acceleration
            self.trajectory_publisher.publish(msg)
            rate.sleep()

        rate.sleep()

        #rospy.loginfo("Finished sending PID-controlled trajectory.")

    def move_to_joint_position_with_spline(self, target_angles, duration=2.0, steps=500):
        if self.current_positions is None:
            rospy.logwarn("Current joint positions not available.")
            return

        steps = (int) (duration * steps)
        start = np.array(self.current_positions)
        end = np.array(target_angles)
        times = np.linspace(0, duration, steps)

        splines = [CubicSpline([0, duration], [start[i], end[i]], bc_type='natural') for i in range(7)]

        rate = rospy.Rate(steps / duration)  # e.g., 50 steps / 2 sec = 250 Hz

        for i, t in enumerate(times):
            position = [float(spline(t)) for spline in splines]
            velocity = [float(spline.derivative(1)(t)) for spline in splines]
            acceleration = [float(spline.derivative(2)(t)) for spline in splines]

            msg = JointCommand()
            msg.header.stamp = rospy.Time.now()
            msg.mode = JointCommand.POSITION_MODE
            msg.names = self.joints
            msg.position = position
            msg.velocity = velocity
            msg.acceleration = acceleration
            # print(np.array(position) - np.array(self.current_positions))

            self.trajectory_publisher.publish(msg)
            rate.sleep()

        # 마지막 보정 (속도=0 보장)
        msg = JointCommand()
        msg.header.stamp = rospy.Time.now()
        msg.mode = JointCommand.POSITION_MODE
        msg.names = self.joints
        msg.position = end.tolist()
        msg.velocity = [0.0] * 7
        msg.acceleration = [0.0] * 7
        self.trajectory_publisher.publish(msg)
        rate.sleep()

        rospy.loginfo("Finished sending spline trajectory.")

    def positions_close_enough(self, current, target):
        if None in current:
            return False
        errors = [abs(c - t) for c, t in zip(current, target)]
        return all(error < self.position_tolerance for error in errors)

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

    def grasp_with_check(self, width, force=5.0, speed=0.1, inner_epsilon=0.02, outer_epsilon=0.01, timeout=5.0):
        goal = GraspGoal()
        goal.width = width
        goal.speed = speed
        goal.force = force
        goal.epsilon.inner = inner_epsilon
        goal.epsilon.outer = outer_epsilon

        rospy.loginfo(f"Sending grasp goal: width={width:.3f}m, force={force:.1f}N")
        self.grasp_client.send_goal(goal)

        if not self.grasp_client.wait_for_result(rospy.Duration(timeout)):
            rospy.logwarn("Grasp action timed out.")
            return False

        result = self.grasp_client.get_result()
        if result.success:
            rospy.loginfo(":white_check_mark: Grasp successful (within epsilon).")
            return True
        else:
            rospy.logwarn(":x: Grasp failed.")
            return False


    def initial_pose(self):

        angles = [0, -0.7, 0, -2.35619449, 0, 1.57079632679, 0.785398163397]
        self.move_to_joint_position(angles)