# #!/usr/bin/env python3
# import rospy
# import pandas as pd

# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from franka_gripper.msg import MoveAction, MoveGoal, GraspAction, GraspGoal, GraspEpsilon
# import actionlib

# def main():
#     # 1) 노드 초기화 & 파라미터 읽기
#     rospy.init_node('csv_trajectory_replay')

#     csv_path = rospy.get_param('~csv_path')
#     rate_hz  = rospy.get_param('~csv_rate', 50.0)
#     joint_names = rospy.get_param('~joint_names', [
#         'panda_joint1','panda_joint2','panda_joint3',
#         'panda_joint4','panda_joint5','panda_joint6','panda_joint7'
#     ])

#     # 2) CSV 로드
#     df = pd.read_csv(csv_path, header=None).values  # shape (T,8)
#     total_steps = df.shape[0]

#     # 3) Topic 퍼블리셔: Position Trajectory Controller
#     traj_pub = rospy.Publisher(
#         '/position_joint_trajectory_controller/command',
#         JointTrajectory,
#         queue_size=1)

#     # 4) 그리퍼 액션 클라이언트
#     move_cli  = actionlib.SimpleActionClient('/franka_gripper/move',  MoveAction)
#     grasp_cli = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
#     rospy.loginfo("Waiting for gripper action servers...")
#     move_cli.wait_for_server()
#     grasp_cli.wait_for_server()

#     # 5) 재생 루프 준비
#     rate = rospy.Rate(rate_hz)
#     last_grip = int(df[0, 7])
#     idx = 0

#     rospy.loginfo("Starting CSV trajectory replay (%d steps at %.1f Hz)...", total_steps, rate_hz)

#     while not rospy.is_shutdown() and idx < total_steps:
#         row = df[idx]
#         idx += 1

#         # --- a) JointTrajectory 메시지 생성 & 퍼블리시 ---
#         jt = JointTrajectory()
#         jt.header.stamp = rospy.Time.now()
#         jt.joint_names = joint_names

#         pt = JointTrajectoryPoint()
#         pt.positions = row[:7].tolist()
#         # 다음 frame까지 걸리는 시간
#         pt.time_from_start = rospy.Duration(1.0 / rate_hz)
#         jt.points = [pt]

#         traj_pub.publish(jt)

#         # --- b) 그리퍼 상태 변화 시 액션 전송 ---
#         grip = int(row[7])
#         if grip != last_grip:
#             if grip == 0:
#                 move_cli.send_goal(MoveGoal(width=0.08, speed=0.1))
#             else:
#                 grasp_cli.send_goal(GraspGoal(
#                     width=0.03,
#                     epsilon=GraspEpsilon(inner=0.01, outer=0.01),
#                     speed=0.1,
#                     force=3.0))
#             last_grip = grip

#         rate.sleep()

#     rospy.loginfo("CSV trajectory replay finished.")

# if __name__ == '__main__':
#     main()



# #!/usr/bin/env python3
# import rospy, pandas as pd, actionlib
# from sensor_msgs.msg    import JointState
# from trajectory_msgs.msg import JointTrajectoryPoint
# from control_msgs.msg    import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
# from franka_gripper.msg  import MoveAction, MoveGoal, GraspAction, GraspGoal, GraspEpsilon

# def main():
#     rospy.init_node('csv_traj_smooth_replay')

#     csv_path = rospy.get_param('~csv_path')
#     rate_hz  = rospy.get_param('~csv_rate', 50.0)
#     joint_names = rospy.get_param('~joint_names', [
#       'panda_joint1','panda_joint2','panda_joint3',
#       'panda_joint4','panda_joint5','panda_joint6','panda_joint7'
#     ])
#     max_dq = rospy.get_param('~max_dq', 0.2)   # rad/s

#     # CSV 로드
#     df = pd.read_csv(csv_path, header=None).values
#     T  = len(df)

#     # Trajectory 액션 클라이언트
#     traj_client = actionlib.SimpleActionClient(
#       '/position_joint_trajectory_controller/follow_joint_trajectory',
#       FollowJointTrajectoryAction)
#     rospy.loginfo("Waiting for traj action server…")
#     traj_client.wait_for_server()

#     # 그리퍼 액션 클라이언트
#     move_cli  = actionlib.SimpleActionClient('/franka_gripper/move',  MoveAction)
#     grasp_cli = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
#     move_cli.wait_for_server(); grasp_cli.wait_for_server()

#     last_grip = int(df[0, 7])
#     rate = rospy.Rate(rate_hz)
#     idx = 0

#     rospy.loginfo("Starting smooth CSV replay…")
#     while not rospy.is_shutdown() and idx < T:
#         row = df[idx]; idx += 1

#         # 1) 현재 상태 읽기
#         js = rospy.wait_for_message(
#             '/franka_state_controller/joint_states', JointState)
#         current = dict(zip(js.name, js.position))

#         # 2) 목표 & 이동 시간 계산
#         target = {jn: row[i] for i, jn in enumerate(joint_names)}
#         max_move = max(abs(target[j] - current[j]) for j in joint_names)
#         move_time = max(max_move / max_dq, 1.0/rate_hz)

#         # 3) Goal 생성
#         pt = JointTrajectoryPoint()
#         pt.positions       = [target[j] for j in joint_names]
#         pt.velocities      = [0.0]*7
#         pt.time_from_start = rospy.Duration(move_time)

#         goal = FollowJointTrajectoryGoal()
#         goal.trajectory.joint_names = joint_names
#         goal.trajectory.points.append(pt)

#         # 4) 보내고 대기
#         traj_client.send_goal(goal)
#         rospy.sleep(move_time)

#         # 5) 그리퍼
#         grip = int(row[7])
#         if grip != last_grip:
#             if grip == 0:
#                 move_cli.send_goal(MoveGoal(width=0.08, speed=0.1))
#             else:
#                 grasp_cli.send_goal(GraspGoal(
#                     width=0.03,
#                     epsilon=GraspEpsilon(inner=0.01,outer=0.01),
#                     speed=0.1, force=3.0))
#             last_grip = grip

#         rate.sleep()

#     rospy.loginfo("Replay done.")

# if __name__=='__main__':
#     main()


# #!/usr/bin/env python3
# import rospy
# import pandas as pd
# from std_msgs.msg       import Float64MultiArray
# from sensor_msgs.msg    import JointState

# # --- (필요 시) gripper 액션 임포트 삭제하거나 유지 ---
# from franka_gripper.msg import MoveAction, MoveGoal, GraspAction, GraspGoal, GraspEpsilon
# import actionlib

# def main():
#     rospy.init_node('csv_torque_replay')

#     # 파라미터
#     csv_path = rospy.get_param('~csv_path')
#     rate_hz  = rospy.get_param('~csv_rate', 50.0)
#     Kp       = rospy.get_param('~Kp', [400,400,400,400,200,200,100])
#     Kd       = rospy.get_param('~Kd', [40,40,40,40,20,20,10])
#     joint_names = rospy.get_param('~joint_names', [
#         'panda_joint1','panda_joint2','panda_joint3',
#         'panda_joint4','panda_joint5','panda_joint6','panda_joint7'
#     ])

#     # CSV 로드
#     df = pd.read_csv(csv_path, header=None).values  # shape (T,8)
#     T  = df.shape[0]

#     # 토픽 퍼블리셔: 토크 명령
#     torque_pub = rospy.Publisher(
#         '/joint_group_effort_controller/command',
#         Float64MultiArray, queue_size=1)

#     # 그리퍼 액션(필요하면)
#     move_cli  = actionlib.SimpleActionClient('/franka_gripper/move',  MoveAction)
#     grasp_cli = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
#     move_cli.wait_for_server(); grasp_cli.wait_for_server()
#     last_grip = int(df[0,7])

#     # 상태 구독용 변수
#     latest_q  = [0.0]*7
#     latest_dq = [0.0]*7
#     def js_cb(msg: JointState):
#         nonlocal latest_q, latest_dq
#         latest_q  = [msg.position[msg.name.index(j)] for j in joint_names]
#         latest_dq = [msg.velocity[msg.name.index(j)] for j in joint_names]

#     rospy.Subscriber('/franka_state_controller/joint_states', JointState, js_cb)

#     rate = rospy.Rate(rate_hz)
#     prev_q_des = [0.0]*7
#     idx = 0

#     rospy.loginfo("Starting CSV torque replay…")
#     while not rospy.is_shutdown() and idx < T:
#         row = df[idx]; idx += 1

#         # desired
#         q_des = row[:7].tolist()
#         # desired velocity: 전 프레임 대비 차분×Hz
#         dq_des = [(q_des[i] - prev_q_des[i]) * rate_hz for i in range(7)]
#         prev_q_des = q_des[:]

#         # PD→τ
#         tau = [ Kp[i]*(q_des[i] - latest_q[i]) + Kd[i]*(dq_des[i] - latest_dq[i])
#                 for i in range(7) ]

#         # 퍼블리시
#         torque_pub.publish(Float64MultiArray(data=tau))

#         # 그리퍼
#         grip = int(row[7])
#         if grip != last_grip:
#             if grip == 0:
#                 move_cli.send_goal(MoveGoal(width=0.08, speed=0.1))
#             else:
#                 grasp_cli.send_goal(GraspGoal(
#                     width=0.03,
#                     epsilon=GraspEpsilon(inner=0.01, outer=0.01),
#                     speed=0.1, force=3.0))
#             last_grip = grip

#         rate.sleep()

#     rospy.loginfo("Replay finished.")

# if __name__ == '__main__':
#     main()


#!/usr/bin/env python3
import rospy
import pandas as pd
from sensor_msgs.msg import JointState

def main():
    rospy.init_node('csv_to_jointstate')

    # 파라미터
    csv_path = rospy.get_param('~csv_path')
    rate_hz  = rospy.get_param('~csv_rate', 50.0)
    joint_names = rospy.get_param('~joint_names', [
        'panda_joint1','panda_joint2','panda_joint3',
        'panda_joint4','panda_joint5','panda_joint6','panda_joint7'
    ])

    # CSV 로드 (7관절 + 그리퍼 상태)
    df = pd.read_csv(csv_path, header=None).values  # (T,8)
    T  = df.shape[0]

    # JointState 퍼블리셔: 임피던스 컨트롤러 입력 토픽으로
    js_pub = rospy.Publisher(
        '/desired_joint_states', JointState, queue_size=1)

    rate = rospy.Rate(rate_hz)
    idx = 0

    rospy.loginfo("Starting CSV -> JointState replay…")
    while not rospy.is_shutdown() and idx < T:
        row = df[idx]; idx += 1

        # 1) JointState 메시지 작성
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = joint_names
        msg.position = row[:7].tolist()
        # 속도 데이터가 있으면 row[7:]로 계산하거나, 없으면 0으로 채웁니다.
        # 여기서는 0으로 고정
        msg.velocity = [0.0]*7

        # 2) 퍼블리시
        js_pub.publish(msg)

        # (Optional) 그리퍼 값 처리: row[7] → 0/1 로직
        # 필요하시면 여기에 gripper 액션 호출을 추가하세요.

        rate.sleep()

    rospy.loginfo("CSV JointState replay finished.")

if __name__ == '__main__':
    main()
