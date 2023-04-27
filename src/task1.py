#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import sqrt, pow, pi

class Task1():
    def callback_function(self, topic_data: Odometry):
        # obtain relevant topic data: pose (position and orientation):
        pose = topic_data.pose.pose
        position = pose.position
        orientation = pose.orientation

        # obtain the robot's position co-ords:
        pos_x = position.x
        pos_y = position.y

        (roll, pitch, yaw) = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w], "sxyz"
        )

        self.x = pos_x
        self.y = pos_y
        self.theta_z = yaw

        # If this is the first time that the callback_function has run
        # (e.g. the first time a message has been received), then
        # obtain a "reference position" (used to determine how far
        # the robot has moved during its current operation)
        if self.startup:
            self.startup = False
            # set the reference position:
            self.x0 = self.x
            self.y0 = self.y
            self.theta_z0 = self.theta_z

    def __init__(self):
        node_name = "Task1"
        self.startup = True
        self.clockwise = False
        self.ctrl_c = False
        rospy.init_node(node_name, anonymous=True)
        
        self.rate = rospy.Rate(1)

        # define the robot pose variables and initialise them to zero:
        # variables for the robot's "current position":
        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        # variables for a "reference position":
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = 0.0

        # setup a '/cmd_vel' publisher and an '/odom' subscriber:
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("odom", Odometry, self.callback_function)
        self.vel = Twist()

        rospy.on_shutdown(self.shutdownhook)
        rospy.loginfo(f"the {node_name} node has been initialised...")

    def shutdownhook(self):
        # publish an empty twist message to stop the robot
        # (by default all velocities will be zero):
        self.pub.publish(Twist())
        self.ctrl_c = True

    def main_loop(self):
        while not self.ctrl_c:
            self.rate.sleep()
            radius = 0.5
            self.vel.linear.x = pi / 30
            self.vel.angular.z = self.vel.linear.x / radius

            StartTime = rospy.get_rostime()

            if self.clockwise:
                self.vel.angular.z *= -1

            self.pub.publish(self.vel)

            while (rospy.get_rostime().secs - StartTime.secs) < 30:
                self.pub.publish(self.vel)
                continue

            if self.clockwise:
                return

            self.clockwise = not self.clockwise


if __name__ == "__main__":
    node = Task1()
    try:
        node.main_loop()
    except rospy.ROSInterruptException:
        pass
