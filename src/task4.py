#!/usr/bin/env python3
import rospy
import actionlib
import math
import cv2
from tb3 import Tb3Odometry
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from tuos_ros_msgs.msg import SearchAction, SearchGoal, SearchFeedback
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion 

class Task4():

    def __init__(self):
        node_name = "Task4"
        rospy.init_node(node_name, anonymous=True)
        self.ctrl_c = False
        self.rate = rospy.Rate(1)

        # initially all positions are 0
        self.odom_tb3 = Tb3Odometry()
        while self.odom_tb3.posx == 0.0 and self.odom_tb3.posy == 0.0:
            rospy.sleep(0.1)
        self.init_x = self.odom_tb3.posx
        self.init_y = self.odom_tb3.posy
        print(f"TurtleBot3's initial position: ({self.odom_tb3.posx}, {self.odom_tb3.posy})")
        

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
        self.goal_sent = False
        self.distance = 0
        self.goal = SearchGoal()

        self.target_colour = ""

        rospy.on_shutdown(self.shutdownhook)
        rospy.loginfo(f"The {node_name} node has been initialised...")

    
    def shutdownhook(self):
        if self.goal_sent:
            self.client.cancel_goal()
            rospy.logwarn("The search action was cancelled.")

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


    def find_target_colour(self):
        speed = 1
        self.vel.angular.z = speed
        time = math.radians(90) / speed
        self.pub.publish(self.vel)
        rospy.sleep(time)
        self.pub.publish(Twist())

        for colour in self.colours:
            mask = cv2.inRange(self.hsv_img, self.colours[colour][0], self.colours[colour][1])
            if mask.any():
                self.target_colour = colour
                print(f"Target colour is {self.target_colour}")
                break

        self.vel.angular.z = -speed
        self.pub.publish(self.vel)
        rospy.sleep(time)
        self.pub.publish(Twist())


    def check_for_target(self):
        mask = cv2.inRange(self.hsv_img, self.colours[self.target_colour][0], self.colours[self.target_colour][1])
        if mask.any():
            print("Target in view")
            return True
        
        return False
        
    


    def main_loop(self):
        while not self.ctrl_c:
            self.rate.sleep()
            rospy.sleep(0.2)
            if self.target_colour == "":
                self.find_target_colour()

             
            #if robots positon >  0.3m from initial position

            #print (self.init_x)
            #print(self.init_y)


            if not self.goal_sent:
                self.goal.approach_distance = 0.4
                self.goal.fwd_velocity = 0.2
                self.client.send_goal(self.goal, feedback_cb=self.server_callback)
                self.goal_sent = True

            #if turtlebot is seeing colour and is 0.3m from initial position
            #if self.check_for_target() and abs(self.odom_tb3.posx - self.init_x) > 0.3 and abs(self.odom_tb3.posy - self.init_y) > 0.3:
                #it moves forwards
                #self.vel.linear.x = 0.5
                #if self.check_for_target() and bot is 10 cm from target:
                #then stop
                    


                # self.goal.approach_distance = 0.2
                # self.goal.fwd_velocity = 0.0
                # self.client.send_goal(self.goal, feedback_cb=self.server_callback)
                # self.goal_sent = True
                #break
            # then move towards target
            # else 
            # continue
            
            
            self.check_for_target()
    
if __name__ == "__main__":
    node = Task4()
    try:
        node.main_loop()
    except rospy.ROSInterruptException:
        pass