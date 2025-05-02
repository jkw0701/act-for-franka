# #!/usr/bin/env python3
# """
# csv_replay_upsampled.py
#   • 30 Hz CSV(7 joint + gripper) → 100 Hz 선형 보간
#   • Float64MultiArray 로 팔 퍼블리시
#   • 그리퍼 서버가 있으면 Move/Grasp; 없으면 건너뜀
# """

# import numpy as np, pandas as pd, rospy, actionlib, pathlib
# from std_msgs.msg import Float64MultiArray
# from franka_gripper.msg import MoveAction, MoveGoal, GraspAction, GraspGoal, GraspEpsilon

# # ---------------- 사용자 파라미터 ----------------
# CSV_PATH   = "~/Real_ACTFranka/real_dir3/csv/episode_32_action.csv"
# CSV_HZ     = 50.0          # 원본 주기 (Hz)
# PUB_HZ     = 200.0         # 보간 후 퍼블리시 주기
# SAFE_VEL   = 1.3           # rad/s  (속도 점검용)
# OPEN_W, CLOSE_W = 0.08, 0.03
# # -------------------------------------------------

# def upsample(mat: np.ndarray, k: int) -> np.ndarray:
#     """row-wise 선형 보간 (k-배)"""
#     out = []
#     for a, b in zip(mat[:-1], mat[1:]):
#         for i in range(k):
#             out.append(a + (b - a) * (i / k))
#     out.append(mat[-1])
#     return np.array(out)

# def main():
#     csv_file = pathlib.Path(CSV_PATH).expanduser().resolve()
#     if not csv_file.exists():
#         raise FileNotFoundError(csv_file)
#     raw = pd.read_csv(csv_file, header=None).values    # (N, 8)

#     k = int(round(PUB_HZ / CSV_HZ))                   # 배수 (30→100 ≈3.33 → 3)
#     up = upsample(raw, k)                             # (N*k, 8)

#     # 속도 jump 안전 점검
#     dt = 1.0 / PUB_HZ
#     max_dq = np.max(np.abs(np.diff(up[:, :7], axis=0)))
#     if max_dq / dt > SAFE_VEL:
#         rospy.logwarn("Upsampled 궤적 속도 %.2f rad/s > SAFE_VEL %.2f; Reflex 위험!",
#                       max_dq/dt, SAFE_VEL)

#     rospy.init_node("csv_replay_upsampled")
#     arm_pub = rospy.Publisher("/joint_group_position_controller/command",
#                               Float64MultiArray, queue_size=1)

#     # 그리퍼 클라이언트 (0.1 s 타임아웃)
#     move_cli  = actionlib.SimpleActionClient("/franka_gripper/move",  MoveAction)
#     grasp_cli = actionlib.SimpleActionClient("/franka_gripper/grasp", GraspAction)
#     GRIPPER_OK = move_cli.wait_for_server(rospy.Duration(0.1)) \
#                  and grasp_cli.wait_for_server(rospy.Duration(0.1))
#     rospy.loginfo("Gripper servers %s", "ready ✓" if GRIPPER_OK else "not found – skip")

#     last_grip = int(up[0, 7])
#     rate = rospy.Rate(PUB_HZ)

#     for row in up:
#         if rospy.is_shutdown():
#             break
#         arm_pub.publish(Float64MultiArray(data=row[:7].tolist()))
#         g = int(row[7])
#         if GRIPPER_OK and g != last_grip:
#             if g == 0:
#                 move_cli.send_goal(MoveGoal(width=OPEN_W, speed=0.1))
#             else:
#                 grasp_cli.send_goal(GraspGoal(width=CLOSE_W,
#                                               epsilon=GraspEpsilon(inner=0.01, outer=0.01),
#                                               speed=0.1, force=3.0))
#             last_grip = g
#         rate.sleep()

#     rospy.loginfo("CSV replay done.")

# if __name__ == "__main__":
#     main()

#!/usr/bin/env python3
import rospy, csv
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

def read_csv(path):
    rows = []
    with open(path, 'r') as f:
        reader = csv.reader(f)
        for line in reader:
            rows.append([float(x) for x in line])
    return rows

if __name__ == "__main__":
    rospy.init_node("csv_to_trajectory")
    # 파라미터
    csv_file   = rospy.get_param("~csv_file")
    joint_names= rospy.get_param("~joint_names")
    rate_hz    = rospy.get_param("~rate", 50.0)
    dt         = 1.0 / rate_hz

    traj = read_csv(csv_file)
    if not traj:
        rospy.logerr("CSV empty!")
        exit(1)

    # 액션 클라이언트 (네임스페이스는 franka_control.launch 설정에 따라 바꿔주세요)
    server_name = "/panda/joint_impedance_controller/follow_joint_trajectory"
    client = actionlib.SimpleActionClient(server_name, FollowJointTrajectoryAction)
    rospy.loginfo(f"Waiting for {server_name}...")
    client.wait_for_server()

    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = joint_names

    # 포인트 생성: 시간 + 위치 + (선택)속도
    t = rospy.Duration(0.0)
    # 속도 보간을 위해 이전 포인트 기억
    prev = traj[0]
    for row in traj:
        p = JointTrajectoryPoint()
        p.positions = row
        # 간단하게도 모두 0속도로 넘기면 controller가 자체 보간해 줍니다.
        # p.velocities = [(row[i]-prev[i])/dt for i in range(len(row))]
        p.time_from_start = t
        goal.trajectory.points.append(p)
        prev = row
        t += rospy.Duration(dt)

    rospy.loginfo("Sending trajectory with %d points...", len(traj))
    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo("Done: %s", client.get_state())
