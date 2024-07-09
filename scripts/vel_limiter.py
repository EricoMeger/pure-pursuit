#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class VelocityLimiter():
    def __init__(self):
        self.vel_sub = rospy.Subscriber("/velocidade", Twist, self.velocidade_callback)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber("/scan_min", Float64, self.scan_callback)

        self.velocidade = Twist()
        self.min_dist = 100000.0
        self.safe_distance = 0.6

    def scan_callback(self, data):
        self.min_dist = data.data

    def velocidade_callback(self, data):
        self.velocidade.linear.x = self.check_velocities(data.linear.x)
        self.velocidade.angular.z = self.check_velocities(data.angular.z)

        if self.min_dist < self.safe_distance:
            self.velocidade.linear.x = 0.0
            self.velocidade.angular.z = 0.0
        elif self.min_dist > 0.6 and self.min_dist < 0.9:
            self.velocidade.linear.x /= 1.6
            self.velocidade.angular.z /= 1.6
        elif self.min_dist > 0.9 and self.min_dist < 1.3:
            self.velocidade.linear.x /= 1.3
            self.velocidade.angular.z /= 1.3

        self.vel_pub.publish(self.velocidade)
       
    def check_velocities(self, vel):
        if vel > 0.4:
            return 0.4
        elif vel < -0.4:
            return -0.4
        else:
            return vel


if __name__ == '__main__':
    rospy.init_node('velocity_limiter')
    VelocityLimiter()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
