#!/usr/bin/env python3
import rospy
import actionlib
import math
import random
import cv2
import numpy as np
from tb3 import Tb3Odometry, Tb3LaserScan, Tb3Move
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, LaserScan
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

        # get initial positions
        self.odom_tb3 = Tb3Odometry()
        while self.odom_tb3.posx == 0.0 and self.odom_tb3.posy == 0.0:
            rospy.sleep(0.1)
        self.init_x = self.odom_tb3.posx
        self.init_y = self.odom_tb3.posy
        print(f"TurtleBot3's initial position: ({self.odom_tb3.posx}, {self.odom_tb3.posy})")
        
        self.target_colour = ""
        self.cam_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.cam_callback)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.cvbridge_interface = CvBridge()
        self.m00 = 0
        self.m00_min = 10000
        # some colour ranges are still incorrect
        self.colours = {
            "blue":       [(115, 225, 100), (130, 255, 130)],
            "green":      [(20, 160, 100), (65, 255, 255)],
            "turqouise":  [(90, 160, 100), (100, 255, 255)],
            "red":        [(0, 205, 100), (10, 255, 255)],
            "yellow":     [(25, 170, 100), (30, 255, 255)],
            "purple":     [(145, 200, 100), (155, 255, 255)]
        }

        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.vel = Twist()
        self.tb3_lidar = Tb3LaserScan()
        self.robot_controller = Tb3Move()

        self.client = actionlib.SimpleActionClient("/obstacle_avoidance_server", SearchAction)
        #self.goal_sent = False
        self.action_complete = False
        self.distance = 0
        self.goal = SearchGoal()

        

        rospy.on_shutdown(self.shutdownhook)
        rospy.loginfo(f"The {node_name} node has been initialised...")

    
    def shutdownhook(self):
        if not self.action_complete:
            self.client.cancel_goal()
            rospy.logwarn("The search action was cancelled.")

        self.pub.publish(Twist())
        self.ctrl_c = True

    def scan_callback(self, scan_data):
        self.scan_data = scan_data.ranges
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
        crop_height = 100
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))

        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        self.hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        if self.target_colour != "":
            self.mask = cv2.inRange(self.hsv_img, self.colours[self.target_colour][0], self.colours[self.target_colour][1])
            m = cv2.moments(self.mask)
            self.m00 = m['m00']
            #the cenre of where the colour is in the screen, 0 is from the right
            self.cy = m['m10'] / (m['m00'] + 1e-5)
            self.cz = m['m01'] / (m['m00'] + 1e-5)

        # cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)

    def server_callback(self, feedback: SearchFeedback):
        self.distance = feedback.current_distance_travelled
        #print(f"Distance travelled: {self.distance}")


    def find_target_colour(self):
        self.vel.linear.x = 0.26
        time = 2
        self.pub.publish(self.vel)
        rospy.sleep(time)

        self.vel.linear.x = 0
        speed = 1
        self.vel.angular.z = speed
        time = math.radians(180) / speed
        self.pub.publish(self.vel)
        rospy.sleep(time)
        self.pub.publish(Twist())

        for colour in self.colours:
            mask = cv2.inRange(self.hsv_img, self.colours[colour][0], self.colours[colour][1])
            if mask.any():
                self.target_colour = colour
                print(f"SEARCH INITIATED: The target beacon colour is {self.target_colour}.")
                break

        self.vel.angular.z = -speed
        self.pub.publish(self.vel)
        rospy.sleep(time)
        self.pub.publish(Twist())


    def check_for_target(self):
        mask = cv2.inRange(self.hsv_img, self.colours[self.target_colour][0], self.colours[self.target_colour][1])
        if mask.any():
            return True
        
        return False

    def turn_around(self):
        while (self.tb3_lidar.min_distance < self.goal.approach_distance 
            and self.tb3_lidar.min_left > 0.175 and self.tb3_lidar.min_right < 0.175):

            self.vel.linear.x = 0
            obj = self.client.get_result()
            speed = 1
            if obj != None:
                obj_angle = obj.closest_object_angle
                if obj_angle > 0:
                    speed = -1

            self.vel.angular.z = speed
            time = math.radians(random.randint(90,180)) / speed
            self.pub.publish(self.vel)
            self.pub.publish(Twist())
            rospy.sleep(time)

            self.vel.linear.x = 0.26
            self.pub.publish(self.vel)

    def main_loop(self):
        #prints coords of the turtlebot
        #print(f"COORDS : ({self.odom_tb3.posx}, {self.odom_tb3.posy})")
        beaconing = False
        found = False
        while not self.ctrl_c:
            self.rate.sleep()
            rospy.sleep(0.2)
            if self.target_colour == "":
                self.find_target_colour()

            if not self.action_complete:
                self.goal.approach_distance = 0.6
                self.goal.fwd_velocity = 0.2
                self.client.send_goal(self.goal, feedback_cb=self.server_callback)
                #self.action_complete = self.client.wait_for_result()
            
            self.check_for_target()
            StartTime = rospy.get_rostime()

            #if turtlebot is seeing colour and is 0.3m from initial position
            if self.check_for_target() and abs(self.odom_tb3.posx - self.init_x) > 0.3 and abs(self.odom_tb3.posy - self.init_y) > 0.3:
                print("TARGET DETECTED: Beaconing initiated.")
                beaconing = True
                self.goal.approach_distance = 0.4

                #if the colour takes up the whole screen ie the object is very close
                # TODO: Adjust conditional to trigger when robot has reached goal state (close to targets)

                if (510 < self.cy < 610 and self.min_dist < self.goal.approach_distance):
                    #stop the robot
                    self.vel.linear.x = 0.0
                    self.vel.angular.z = 0.0                        
                    self.pub.publish(self.vel)
                    print("BEACONING COMPLETE: The robot has now stopped.")
                    self.pub.publish(Twist())
                    found = True
                    break

                # if the target colour is in the middle
                if 510 < self.cy < 610:
                    #print("moving fwd")
                    while(rospy.get_rostime().secs - StartTime.secs) < 2:
                        # move fwd
                        self.vel.linear.x = 0.26
                        self.vel.angular.z = 0.0
                        
                        self.pub.publish(self.vel)
                        #self.robot_controller.set_move_cmd(0.5, 0.0)
                        
                # if the target colour is on the right hand side
                elif self.cy >= 610:
                    #print("moving right")
                    while(rospy.get_rostime().secs - StartTime.secs) < 0.1:
                        # move right
                        self.vel.angular.z = -0.1
                        self.vel.linear.x = 0.0
                        self.pub.publish(self.vel)

                        #self.robot_controller.set_move_cmd(0.0, -0.5)
                    #self.pub.publish(Twist())

                #if target colour is on the left hand side
                elif self.cy <= 510:
                    #print("moving left")
                    while(rospy.get_rostime().secs - StartTime.secs) < 0.1:
                        #move left
                        self.vel.angular.z = 0.1
                        self.vel.linear.x = 0.0
                        self.pub.publish(self.vel)
                        #self.robot_controller.set_move_cmd(0.0, -0.5)
                    #self.pub.publish(Twist())
            if found == True:
                self.vel.linear.x = 0.0
                self.vel.angular.z = 0.0
                self.pub.publish(self.vel)
                break
            if beaconing == False and found == False:
                self.turn_around()

#roslaunch com2009_simulations beaconing.launch start_zone:=c

if __name__ == "__main__":
    node = Task4()
    try:
        node.main_loop()
    except rospy.ROSInterruptException:
        pass