#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
import math
from pose_handler import PoseHandler

class PathFollower():
    def __init__(self):
        self.path_sub = rospy.Subscriber("/move_base/GlobalPlanner/global_plan", Path, self.path_callback)
        self.cmdvel_pub = rospy.Publisher("/velocidade", Twist, queue_size=10)

        self.position_handler = PoseHandler()
        self.path = None
        self.velocidade = Twist()

        self.index = 0
        self.lookahead_distance = 1.0 
        self.rotate_in_axis = False 
    
        rospy.on_shutdown(self.stop_robot)

    def publish_velocidade(self, x, z):
        self.velocidade.linear.x = x
        self.velocidade.angular.z = z

        self.cmdvel_pub.publish(self.velocidade)

    def stop_robot(self):
        self.publish_velocidade(0.0, 0.0)

    def path_callback(self, data):
        print("received path")
        self.path = data.poses
        self.index = 0 

    def find_lookahead_point(self):
        if not self.path or self.index >= len(self.path):
            return None

        for i in range(self.index, len(self.path)):
            pos_x = self.path[i].pose.position.x
            pos_y = self.path[i].pose.position.y
            distance = math.sqrt((pos_x - self.position_handler.robot_pose[0]) ** 2 + 
                                 (pos_y - self.position_handler.robot_pose[1]) ** 2)
            if distance >= self.lookahead_distance:
                self.index = i
                return pos_x, pos_y
        return None

    def check_angle(self, angle):
        if abs(angle) > math.pi / 2 and abs(angle) < 3 * math.pi / 2:
            self.rotate_in_axis = True
            print("the goal is behind the robot")

    def align_with_target(self, current_angle, target_angle, threshold=0.15):
        angle_difference = self.normalize_angle(target_angle - current_angle)
        
        if abs(angle_difference) > threshold:
            if angle_difference > 0:
                self.rotate_left()
            else:
                self.rotate_right()
        else:
            self.stop_robot()
            self.rotate_in_axis = False
            print("Aligned with the target")

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def rotate_left(self):
        self.publish_velocidade(0, 0.3)

    def rotate_right(self):
        self.publish_velocidade(0, -0.3)

    def move_robot(self):
        if not self.path or self.index >= len(self.path):
            self.stop_robot()
            return

        lookahead_point = self.find_lookahead_point()
        if lookahead_point:
            pos_x, pos_y = lookahead_point

            delta_x = pos_x - self.position_handler.robot_pose[0]
            delta_y = pos_y - self.position_handler.robot_pose[1]

            goal_angle = math.atan2(delta_y, delta_x)
            angular_diff = goal_angle - self.position_handler.robot_orientation[2]
            
            angular_diff = self.normalize_angle(angular_diff)

            self.check_angle(angular_diff)

            if self.rotate_in_axis:
                self.align_with_target(self.position_handler.robot_orientation[2], goal_angle)
            else:
                self.publish_velocidade(0.4, 2 * angular_diff)

            if self.position_handler.has_reached(pos_x, pos_y):
                self.index += 1

if __name__ == '__main__':
    rospy.init_node("path_follower")
    rate = rospy.Rate(10)
    objt = PathFollower()
    print("Initializing...")
    while not rospy.is_shutdown():
        objt.move_robot()
        rate.sleep()
