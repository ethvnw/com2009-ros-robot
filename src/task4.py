#!/usr/bin/env python3
import rospy
import actionlib
import math
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tuos_ros_msgs.msg import SearchAction, SearchGoal, SearchFeedback

class Task4():

    def __init__(self):
        node_name = "Task4"
        rospy.init_node(node_name, anonymous=True)
        self.ctrl_c = False
        self.rate = rospy.Rate(1)

        # self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.cam_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.cam_callback)
        self.cvbridge_interface = CvBridge()
        self.m00 = 0
        self.m00_min = 10000
        self.colours = {
            "blue":       [(115, 225, 100), (130, 255, 255)],
            "green":      [(20, 160, 100), (65, 255, 255)],
            "turqouise":  [(90, 160, 100), (100, 255, 255)],
            "red":        [(0, 205, 100), (10, 255, 255)],
            "yellow":     [(25, 170, 100), (30, 255, 255)],
            "purple":     [(145, 200, 100), (155, 255, 255)]
        }

        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.vel = Twist()

        self.client = actionlib.SimpleActionClient("/obstacle_avoidance_server", SearchAction)
        self.action_complete = False
        self.distance = 0
        self.goal = SearchGoal()

        self.target_colour = ""

        rospy.on_shutdown(self.shutdownhook)
        rospy.loginfo(f"The {node_name} node has been initialised...")

    
    def shutdownhook(self):
        if not self.action_complete:
            self.client.cancel_goal()
            rospy.logwarn("The search action was cancelled.")

        self.pub.publish(Twist())
        self.ctrl_c = True

    def scan_callback(self, scan_data):
        left_arc = scan_data.ranges[0:21]
        right_arc = scan_data.ranges[-20:]
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        self.min_dist = front_arc.min()

        arc_angles = np.arange(-20, 21)
        self.obj_angle = arc_angles[np.argmin(front_arc)]

    def cam_callback(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        height, width, _ = cv_img.shape
        crop_width = width - 800
        crop_height = 400
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))

        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        self.hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        if self.target_colour != "":
            self.mask = cv2.inRange(self.hsv_img, self.colours[self.target_colour][0], self.colours[self.target_colour][1])
            m = cv2.moments(self.mask)
            self.m00 = m['m00']
            self.cy = m['m10'] / (m['m00'] + 1e-5)
            self.cz = m['m01'] / (m['m00'] + 1e-5)
            
        # cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)

    def server_callback(self, feedback: SearchFeedback):
        self.distance = feedback.current_distance_travelled
        print(f"Distance travelled: {self.distance}")

    def find_target_colour(self):
        self.vel.angular.z = 0.6
        time = math.radians(90) / 0.6
        self.pub.publish(self.vel)
        rospy.sleep(time)
        self.pub.publish(Twist())

        for colour in self.colours:
            mask = cv2.inRange(self.hsv_img, self.colours[colour][0], self.colours[colour][1])
            if mask.any():
                self.target_colour = colour
                print(f"Target colour is {self.target_colour}")
                break

        self.vel.angular.z = -0.6
        self.pub.publish(self.vel)
        rospy.sleep(time)
        self.pub.publish(Twist())


    def main_loop(self):
        while not self.ctrl_c:
            self.rate.sleep()
            rospy.sleep(0.2)

            # if self.target_colour == "":
            #     rospy.sleep(0.5)
            #     self.find_target_colour()


            if not self.action_complete:
                self.goal.approach_distance = 0.4
                self.goal.fwd_velocity = 0.15
                self.client.send_goal(self.goal, feedback_cb=self.server_callback)
                self.action_complete = self.client.wait_for_result()

            if self.action_complete:
                print("action complete")
                self.shutdownhook()
    
if __name__ == "__main__":
    node = Task4()
    try:
        node.main_loop()
    except rospy.ROSInterruptException:
        pass