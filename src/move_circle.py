#!/usr/bin/env python3
# A simple ROS publisher node in Python

import rospy 
from geometry_msgs.msg import Twist

class Circle(): 

    def __init__(self): 
        self.node_name = "move_circle" 
        topic_name = "cmd_vel" 
        self.vel_cmd = Twist()

        self.pub = rospy.Publisher(topic_name, Twist, queue_size=10) 
        rospy.init_node(self.node_name, anonymous=True) 
        self.rate = rospy.Rate(10) 

        self.ctrl_c = False 
        rospy.on_shutdown(self.shutdownhook) 

        rospy.loginfo(f"The '{self.node_name}' node is active...") 

    def shutdownhook(self): 
        self.vel_cmd.linear.x = 0.0
        self.vel_cmd.angular.z = 0.0
        self.pub.publish(self.vel_cmd)
        self.ctrl_c = True

    def main_loop(self):
        while not self.ctrl_c: 
            self.vel_cmd.linear.x = 0.1
            radius = 0.5
            self.vel_cmd.angular.z = self.vel_cmd.linear.x / radius
            self.pub.publish(self.vel_cmd)
            self.rate.sleep()

if __name__ == '__main__': 
    publisher_instance = Circle() 
    try:
        publisher_instance.main_loop() 
    except rospy.ROSInterruptException:
        pass
