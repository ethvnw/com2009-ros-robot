#!/usr/bin/env python3

import rospy
# import the Odometry message from the nav_msgs package:
from nav_msgs.msg import Odometry
# additionally, import the "euler_from_quaternion" function from the tf library
# for converting the raw orientation values from the odometry message into euler angles:
from tf.transformations import euler_from_quaternion 
from math import pi

class OdometryTask1():
    def callback(self, topic_data: Odometry):
        # We're only interested in the pose part of the Odometry message,
        pose = topic_data.pose.pose

        # This contains information about both the "position" and "orientation"
        # of the robot, so let's extract those two parts out next:
        position = pose.position
        orientation = pose.orientation

        # "position" data is provided in meters, so we don't need to do any
        # conversion on this, and can extract the relevant parts of this directly:
        pos_x = position.x 
        pos_y = position.y
        pos_z = position.z

        # "orientation" data is in quaternions, so we need to convert this 
        # using the "euler_from_quaternion" function 
        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, 
                        orientation.y, orientation.z, orientation.w], 
                        'sxyz')

        # to convert yaw into degrees = radians * (180/pi)
        yaw_to_degrees = yaw * (180 / pi)

        # Here we print out the values that we're interested in:
        if self.counter > 10:
            self.counter = 0
            print(f"x = {pos_x:.3f} (m), y = {pos_y:.3f} (m), yaw = {abs(yaw_to_degrees):.1f} (radians)")
        else:
            self.counter += 1

    def __init__(self):
        node_name = "odom_task1" # a name for our node (we can call it anything we like)
        rospy.init_node(node_name, anonymous=True)

        self.sub = rospy.Subscriber("odom", Odometry, self.callback)
        rospy.loginfo(f"The '{node_name}' node is active...")
        
        self.counter = 0

    def main_loop(self):
        # set the node to remain active until closed manually:
        rospy.spin()

if __name__ == '__main__':
    subscriber_instance = OdometryTask1()
    subscriber_instance.main_loop()