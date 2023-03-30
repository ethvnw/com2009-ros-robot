#!/usr/bin/env python3
# A simple ROS publisher node in Python

import math
import time
import rospy 
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tb3 import Tb3Odometry

class Task1(): 

    def __init__(self): 
        self.node_name = "move_feight" 
        topic_name = "cmd_vel" 
        self.vel_cmd = Twist()
        self.odom = Tb3Odometry()

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
            self.vel_cmd.linear.x = math.pi / 30
            radius = 0.5
            self.vel_cmd.angular.z  = self.vel_cmd.linear.x / radius
            initial_x = self.odom.posx
            initial_y = self.odom.posy
            prev_yaw = int(self.odom.yaw)
            self.pub.publish(self.vel_cmd)
            print("naptime")
            time.sleep(30)
            print("naptime over")

            self.vel_cmd.angular.z *= -1
            self.pub.publish(self.vel_cmd)




            #publisher_message = f"rospy time is: {rospy.get_time()}"
            # self.rate.sleep()



            # while prev_yaw >= 0 and int(self.odom.yaw) >= 0:
            #     prev_yaw = int(self.odom.yaw)
            #     print(int(self.odom.yaw))
                
            #     continue

            # self.vel_cmd.angular.z *= -1
            # self.pub.publish(self.vel_cmd)
            # self.rate.sleep()

            # while prev_yaw >= 0 and int(self.odom.yaw) >= 0:
            #     prev_yaw = int(self.odom.yaw)
            #     print(int(self.odom.yaw))
                
            #     continue

            # self.vel_cmd.linear.x = 0
            # self.vel_cmd.angular.z = 0
            # self.pub.publish(self.vel_cmd)
            # self.rate.sleep()

if __name__ == '__main__': 
    publisher_instance = Task1() 
    try:
        publisher_instance.main_loop()
    except rospy.ROSInterruptException:
        pass
