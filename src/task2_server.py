#! /usr/bin/env python3
# task2_server.py

#run commands:

#roslaunch com2009_simulations obstacle_avoidance.launch
#rosrun com2009_team5 task2_server.py
#rosrun com2009_team5 task2.py

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import actionlib
from geometry_msgs.msg import Twist
from tuos_ros_msgs.msg import SearchAction, SearchFeedback, SearchResult, SearchGoal
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan

# Import some other useful Python Modules
from math import sqrt, pow

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
        self.actionserver.start()
        rospy.loginfo("The 'Search Action Server' is active...")

        # pull in some useful publisher/subscriber functions from the tb3.py module:
        self.vel_controller = Tb3Move()
        self.tb3_odom = Tb3Odometry()
        self.tb3_lidar = Tb3LaserScan()

    # The action's "callback function":
    def action_server_launcher(self, goal: SearchGoal):
        rate = rospy.Rate(10)
        vel = goal.fwd_velocity #m/s
        dist = goal.approach_distance #m
        success = True

        if vel > 0.26 or vel <= 0:
            print("velocity out of range")
            success = False

        if not success:
            self.actionserver.set_aborted(self.result)
            return

        #print(f"Search goal recieved: fwd_vel = {vel} m/s, approach_distance = {dist} m.")

        # Get the robot's current odometry from the Tb3Odometry() class:
        self.posx0 = self.tb3_odom.posx
        self.posy0 = self.tb3_odom.posy
        # Get information about objects up ahead from the Tb3LaserScan() class:
        self.closest_object = self.tb3_lidar.min_distance
        self.closest_object_location = self.tb3_lidar.closest_object_position

        self.vel_controller.set_move_cmd(vel)

        while self.closest_object > dist:
            # print("closest less than dist")
            # print(self.closest_object)

            if self.actionserver.is_preempt_requested():
                rospy.loginfo("Action has been stoped before completion.")
                self.vel_controller.stop()
                success = False
                self.actionserver.set_preempted(self.result)
                break

            self.update_odom()
            self.vel_controller.publish()
            rate.sleep()

        #self.vel_controller.stop()
        self.vel_controller.set_move_cmd(0,0)
        self.vel_controller.publish()
        # print("outside of colsit dist loop")

        if success:
            self.actionserver.set_succeeded(self.result)
            #self.vel_controller.stop()           
            success = True
            # print("success")
        else:
            rospy.loginfo("run unsuccessful.")
            self.result.total_distance_travelled = -1.0
            self.result.closest_object_angle = -1.0
            self.closest_object_distance = -1.0
            self.actionserver.set_aborted(self.result)
            self.vel_controller.stop()
            return

    def update_odom(self):
        self.distance = sqrt(pow(self.posx0 - self.tb3_odom.posx, 2) + pow(self.posy0 - self.tb3_odom.posy, 2))
        self.feedback.current_distance_travelled = self.distance
    
        self.closest_object = self.tb3_lidar.min_distance
        self.closest_object_location = self.tb3_lidar.closest_object_position
        
        self.result.closest_object_distance = self.closest_object
        self.result.closest_object_angle = self.closest_object_location
        self.result.total_distance_travelled = self.feedback.current_distance_travelled
        
        self.actionserver.publish_feedback(self.feedback)

if __name__ == '__main__':
    rospy.init_node("search_action_server")
    SearchActionServer()
    rospy.spin()