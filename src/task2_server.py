#! /usr/bin/env python3
# search_server.py

#run commands:
#roslaunch com2009_simulations obstacle_avoidance.launch
#rosrun com2009_team5 task2_server.py
#rosrun com2009_team5 task2.py

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import actionlib
from scipy.stats import levy
import random

# Import all the necessary ROS message types:
from geometry_msgs.msg import Twist
from tuos_ros_msgs.msg import SearchAction, SearchFeedback, SearchResult, SearchGoal

# Import the tb3 modules from tb3.py
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan

# Import some other useful Python Modules
from math import sqrt, pow, radians

class SearchActionServer():
    feedback = SearchFeedback() 
    result = SearchResult()

    def __init__(self):
        self.actionserver = actionlib.SimpleActionServer(
            "/com2009_team5_task2_server",
            SearchAction,
            self.action_server_launcher,
            auto_start = False
        )
        # pull in some useful publisher/subscriber functions from the tb3.py module:
        self.vel_controller = Tb3Move()
        self.tb3_odom = Tb3Odometry()
        self.tb3_lidar = Tb3LaserScan()

        self.actionserver.start()
        rospy.loginfo("The 'Search Action Server' is active...")

    # The action's "callback function":
    def action_server_launcher(self, goal: SearchGoal):
        rate = rospy.Rate(10)
        vel = goal.fwd_velocity #m/s
        dist = goal.approach_distance #m
        success = False

        print(f"Search goal recieved: fwd_vel = {vel} m/s, approach_distance = {dist} m.")

        # Get the robot's current odometry from the Tb3Odometry() class:
        self.posx0 = self.tb3_odom.posx
        self.posy0 = self.tb3_odom.posy
        # Get information about objects up ahead from the Tb3LaserScan() class:
        self.closest_object = self.tb3_lidar.min_distance
        self.closest_object_location = self.tb3_lidar.closest_object_position
        self.prev_posx = self.posx0
        self.prev_posy = self.posy0
        self.update_odom()

        if vel > 0.26 or vel < 0:
            print("velocity out of range")
            success = False

        if dist < 0.2:
            print("invalid distance")
            success = False

        while not self.actionserver.is_preempt_requested():
            self.vel_controller.set_move_cmd(linear = vel, angular = 0.0)
            self.vel_controller.publish()

            while self.closest_object > dist:
                print("too close")
                self.update_odom()
                rate.sleep()
                #TODO: add in turn ability when too close, rather than stop

            self.vel_controller.stop()

            while self.closest_object < dist:
                print("moving fwd")
                # update LaserScan data:
                levy_number = levy.rvs(scale=0.1,size=1)[0]

                self.closest_object = self.tb3_lidar.min_distance
                self.closest_object_location = self.tb3_lidar.closest_object_position

                #TODO: look into time going backwards error caused here vvv
                startTime = rospy.get_rostime()            

                while ((rospy.get_rostime().secs - startTime.secs) < levy_number) and (self.closest_object > dist):
                    self.update_odom()

                    #TODO: check if levy element acc works
                    self.vel_controller.set_move_cmd(vel,0)
                self.vel_controller.publish()

            if success:
                rospy.loginfo("approach completed successfully.")
                print("success")
                self.vel_controller.stop()
                self.actionserver.set_succeeded(self.result)
            else:
                print("no success")
                self.result.total_distance_travelled = -1.0
                self.result.closest_object_angle = -1.0
                self.closest_object_distance = -1.0
                self.actionserver.set_aborted(self.result)
                return
        
        self.actionserver.set_preempted()
        self.vel_controller.stop()           
        success = False
        print("The action has been preempted.")

    def update_odom(self):
        self.feedback.current_distance_travelled += sqrt(pow(self.prev_posx - self.tb3_odom.posx, 2) + pow(self.prev_posy - self.tb3_odom.posy, 2))
        self.prev_posx = self.tb3_odom.posx
        self.prev_posy = self.tb3_odom.posy
    
        self.closest_object = self.tb3_lidar.min_distance
        self.closest_object_location = self.tb3_lidar.closest_object_position
        self.distance = sqrt(pow(self.posx0 - self.tb3_odom.posx, 2) + pow(self.posy0 - self.tb3_odom.posy, 2))


        self.result.total_distance_travelled = self.feedback.current_distance_travelled
        self.result.closest_object_distance = self.closest_object
        self.result.closest_object_angle = self.closest_object_location
        self.actionserver.publish_feedback(self.feedback)

if __name__ == '__main__':
    rospy.init_node("search_action_server")
    SearchActionServer()
    rospy.spin()