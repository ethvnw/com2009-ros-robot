#!/usr/bin/env python3
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class Task4():

    def __init__(self):
        node_name = "Task4"
        rospy.init_node(node_name, anonymous=True)
        self.ctrl_c = False
        self.rate = rospy.Rate(1)

        self.cam_sub = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.cam_callback)
        self.cvbridge_interface = CvBridge()

        self.colour = 0
        self.finding_target = False
        self.target_colour = ""
        self.target_found = False
        self.m00 = 0
        self.m00_min = 10000


        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.vel = Twist()


        rospy.on_shutdown(self.shutdownhook)
        rospy.loginfo(f"the {node_name} node has been initialised...")

    
    def shutdownhook(self):
        # publish an empty twist message to stop the robot
        # (by default all velocities will be zero):
        self.pub.publish(Twist())
        self.ctrl_c = True


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
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)


        colours = ["blue", "green", "turqouise", "red", "yellow", "purple"]
        lowers = [(115, 225, 100), (50, 225, 100), (90, 162, 100), (0, 205, 100), (25, 155, 100), (147, 200, 100)]
        uppers = [(130, 255, 255), (65, 255, 255), (91, 255, 255), (0, 255, 255), (30, 255, 255), (154, 255, 255)]

        
        mask = cv2.inRange(hsv_img, lowers[self.colour], uppers[self.colour])
        res = cv2.bitwise_and(crop_img, crop_img, mask = mask)


        m = cv2.moments(mask)
        self.m00 = m['m00']
        self.cy = m['m10'] / (m['m00'] + 1e-5)

        if self.m00 > self.m00_min:
            if self.finding_target:
                self.target_found = True
                self.finding_target = False
                self.target_colour = colours[self.colour]
                cv2.imshow('cropped image', res)
        
        cv2.waitKey(1)


    def find_target_colour(self):
        t0 = rospy.get_rostime().secs
        self.vel.angular.z = 0.6

        while (rospy.get_rostime().secs - t0) < 4:
            self.pub.publish(self.vel)

        self.pub.publish(Twist())

        self.finding_target = True
        while not self.target_found:
            self.colour +=1 
            if self.colour >= 5:
                self.colour = 0
        

        t0 = rospy.get_rostime().secs
        self.vel.angular.z = -0.6

        while (rospy.get_rostime().secs - t0) < 3.05:
            self.pub.publish(self.vel)

        self.pub.publish(Twist())


    def main_loop(self):
        while not self.ctrl_c:
            if not self.target_found:
                self.find_target_colour()

            print(self.target_colour)




    
if __name__ == "__main__":
    node = Task4()
    try:
        node.main_loop()
    except rospy.ROSInterruptException:
        pass