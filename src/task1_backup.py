#!/usr/bin/env python3
# A simple ROS publisher node in Python

import rospy 
import math
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class Task1(): 

    def __init__(self): 
        self.node_name = "move_feight" 
        topic_name = "cmd_vel" 
        self.vel_cmd = Twist()

        self.pub = rospy.Publisher(topic_name, Twist, queue_size=10) 
        rospy.init_node(self.node_name, anonymous=True) 
        self.rate = rospy.Rate(10) 

        self.ctrl_c = False 
        rospy.on_shutdown(self.shutdownhook) 

        rospy.loginfo(f"The '{self.node_name}' node is active...") 

    def shutdownhook(self): 
        print(f"Stopping the '{self.node_name}' node at: {rospy.get_time()}")
        self.ctrl_c = True

    def main_loop(self):
        while not self.ctrl_c:
            radius = 0.5
            time = 30
            self.vel_cmd.angular.z  = math.pi * 2 / time
            self.vel_cmd.linear.x = self.vel_cmd.angular.z * radius
            #publisher_message = f"rospy time is: {rospy.get_time()}"
            self.pub.publish(self.vel_cmd)
            self.rate.sleep()

if __name__ == '__main__': 
    publisher_instance = Task1() 
    try:
        publisher_instance.main_loop()
    except rospy.ROSInterruptException:
        pass
