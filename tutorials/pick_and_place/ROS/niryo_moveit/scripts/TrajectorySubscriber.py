#!/usr/bin/env python
"""
    Subscribes to TargetDestination topic.
    Uses MoveIt to compute a trajectory from the target to the destination.
    Trajectory is then published to PickAndPlaceTrajectory topic.
"""
import rospy

from niryo_moveit.msg import NiryoMoveitJoints, NiryoTrajectory
from moveit_msgs.msg import RobotTrajectory


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard:\npick_pose:\n%s\nplace_pose\n%s", data.pick_pose, data.place_pose)

    pub = rospy.Publisher('NiryoTrajectory', NiryoTrajectory, queue_size=10)
    trajectories = NiryoTrajectory()
    robo_traj = RobotTrajectory()
    robo_traj.joint_trajectory.joint_names = ['one', 'two', 'blah']
    trajectories.trajectory.append(robo_traj)

    print(trajectories)
    pub.publish(trajectories)


def listener():
    rospy.init_node('Trajectory_Subscriber', anonymous=True)
    rospy.Subscriber("SourceDestination", NiryoMoveitJoints, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
