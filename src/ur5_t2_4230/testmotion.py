#!/usr/bin/python
#
# Send joint values to UR5 using messages
#

from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from tf.transformations import quaternion_from_euler
import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

#waypoints = [[0.0, -1.44, 1.4, 0.6, 0, -0.33], [0,0,0,0,0,0]]

def main():
    robot = moveit_commander.RobotCommander()

    group = moveit_commander.MoveGroupCommander("manipulator")
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = 1.0
    pose_target.position.x = 0.2
    pose_target.position.y = 0.6
    pose_target.position.z = 0.25

    # Plans path to avoid collision objects in Planning Scene:
    group.set_planner_id("RRTConnectkConfigDefault")
    group.set_pose_target(pose_target)
    group.plan()
    group.go(wait=True)
    rospy.spin()

    """
    rospy.init_node('send_joints')
    pub = rospy.Publisher('/arm_controller/command',
                          JointTrajectory,
                          queue_size=10)

    # Create the topic message
    traj = JointTrajectory()
    traj.header = Header()
    # Joint names for UR5
    traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                        'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                        'wrist_3_joint']

    rate = rospy.Rate(1)
    cnt = 0
    pts = JointTrajectoryPoint()
    traj.header.stamp = rospy.Time.now()

    while not rospy.is_shutdown():
        cnt += 1

        if cnt%2 == 1:
            pts.positions = waypoints[0]
        else:
            pts.positions = waypoints[1]

        pts.time_from_start = rospy.Duration(1.0)

        # Set the points to the trajectory
        traj.points = []
        traj.points.append(pts)
        # Publish the message
        pub.publish(traj)
        rate.sleep()
        """
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")
