"""
csv_continuous_publisher.py

Reads a CSV of joint positions (7 cols) + gripper flag (8th col)
and publishes each row at a fixed rate as a JointState to drive
continuous_move_to_start.py, also sending gripper actions on changes.

Parameters (private ns):
  csv_path      : path to CSV file (no header, 8 columns)
  publish_hz    : publish frequency in Hz (default: 50.0)
  target_topic  : JointState topic to publish to (default: /target_joint_states)
  joint_states  : topic name for current states (unused here)
  joint_names   : list of 7 joint names (default Panda joints)
  open_width    : gripper open width (m, default 0.08)
  close_width   : gripper close width (m, default 0.03)
  gripper_move_speed : speed for gripper move (default 0.1)
  gripper_grasp_speed: speed for grasp (default 0.1)
  gripper_force      : force for grasp (default 3.0)

Publishes:
  sensor_msgs/JointState to target_topic

Uses:
  /franka_gripper/move  (MoveAction)
  /franka_gripper/grasp (GraspAction)
"""
import sys
import rospy
import pandas as pd
from sensor_msgs.msg import JointState
from franka_gripper.msg import MoveAction, MoveGoal, GraspAction, GraspGoal, GraspEpsilon
import actionlib

def main():
    rospy.init_node('csv_continuous_publisher')

    # Params
    csv_path = rospy.get_param('~csv_path', '~/Real_ACTFranka/real_dir3/csv/episode_2_action.csv')
    publish_hz = rospy.get_param('~publish_hz', 40.0)
    target_topic = rospy.get_param('~target_topic', '/target_joint_states')
    joint_names = rospy.get_param('~joint_names', [
        'panda_joint1','panda_joint2','panda_joint3','panda_joint4',
        'panda_joint5','panda_joint6','panda_joint7'])
    open_w = rospy.get_param('~open_width', 0.08)
    close_w = rospy.get_param('~close_width', 0.025)
    move_speed = rospy.get_param('~gripper_move_speed', 0.1)
    grasp_speed = rospy.get_param('~gripper_grasp_speed', 0.1)
    grasp_force = rospy.get_param('~gripper_force', 10.0)

    if not csv_path:
        rospy.logerr('csv_continuous_publisher: ~csv_path param is empty')
        sys.exit(1)

    # Load CSV
    try:
        df = pd.read_csv(csv_path, header=None).values
    except Exception as e:
        rospy.logerr('Failed to load CSV: %s', e)
        sys.exit(1)

    # Publisher
    pub = rospy.Publisher(target_topic, JointState, queue_size=1)

    # Gripper action clients
    move_client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
    grasp_client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
    rospy.loginfo('Waiting for gripper actions...')
    move_client.wait_for_server()
    grasp_client.wait_for_server()

    # Initialize last grip state
    last_grip = int(df[0,7])

    rate = rospy.Rate(publish_hz)
    total = df.shape[0]
    rospy.loginfo('Starting CSV continuous publish: %d steps at %.1f Hz', total, publish_hz)

    idx = 0
    while not rospy.is_shutdown() and idx < total:
        row = df[idx]
        # 1) Publish JointState
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = joint_names
        js.position = row[:7].tolist()
        pub.publish(js)

        # 2) Gripper action on change
        g = int(row[7])
        if g != last_grip:
            if g == 0:
                move_client.send_goal(MoveGoal(width=open_w, speed=move_speed))
            else:
                grasp_client.send_goal(GraspGoal(
                    width=close_w,
                    epsilon=GraspEpsilon(inner=0.013, outer=0.013),
                    speed=grasp_speed,
                    force=grasp_force))
            last_grip = g

        rospy.loginfo('Step %d/%d published, gripper=%d', idx+1, total, g)
        idx += 1
        rate.sleep()

    rospy.loginfo('CSV continuous publish finished.')

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
