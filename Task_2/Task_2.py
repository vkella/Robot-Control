#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from Task_1 import *


class AngleCorrectionNode:
    def __init__(self):
        rospy.init_node('angle_correction', anonymous=True)

        # Creating a publisher for the cmd_vel topic
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


    def publish_cmd_vel(self, correction_angle):

        # Read the point cloud data and calucalte the angle 
       pointcloud = read_pointcloud("4.npz")
       lines = line_detection(pointcloud)
       angle=calculate_angle(lines)

       if angle >0 :
           
           correction_angle = -90+angle
       else:
         
           correction_angle= 90+angle
        
        # Creating a Twist message
        twist_msg = Twist()
        twist_msg.angular.z = correction_angle

        # Publish the Twist message to the cmd_vel topic
        self.cmd_vel_pub.publish(twist_msg)

    def run(self):
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.publish_cmd_vel()
            rate.sleep()

if __name__ == '__main__':
    try:
        angle_node = AngleCorrectionNode()
        angle_node.run()
        
    except rospy.ROSInterruptException:
        pass
