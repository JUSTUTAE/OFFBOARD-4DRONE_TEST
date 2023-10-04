#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State

current_pose = PoseStamped()
leader_target_x = 8.0
leader_target_y = 3.0
leader_target_z = 5.0

def pose_cb(msg):
    global current_pose
    current_pose = msg

if __name__ == "__main__":
    rospy.init_node("error_monitor")

    pose_sub = rospy.Subscriber("uav0/mavros/local_position/pose", PoseStamped, callback=pose_cb)
    rate = rospy.Rate(10)  # 10 Hz (0.1 seconds)

    while not rospy.is_shutdown():
        error_x = leader_target_x - current_pose.pose.position.x
        error_y = leader_target_y - current_pose.pose.position.y
        error_z = leader_target_z - current_pose.pose.position.z

        rospy.loginfo(f"오차 (x, y, z): ({error_x}, {error_y}, {error_z})")

        rate.sleep()

