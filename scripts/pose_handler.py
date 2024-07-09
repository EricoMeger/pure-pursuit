#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf

class PoseHandler():
    def __init__(self):
        self.pose_callback = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)

        self.robot_pose = [0, 0]
        self.robot_orientation = [0, 0, 0]

    def pose_callback(self, data):
        self.robot_pose[0] = data.pose.pose.position.x
        self.robot_pose[1] = data.pose.pose.position.y
        self.robot_orientation = (tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]))

    def has_reached(self, x, y):
        if (abs(self.robot_pose[0] - x) < 0.1 and abs(self.robot_pose[1] - y) < 0.1):
            return True
        else:
            return False