#! /usr/bin/env python3
# search_server.py

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import actionlib

# Import all the necessary ROS message types:
from tuos_ros_msgs.msg import SearchAction, SearchFeedback, SearchResult, SearchGoal

# Import the tb3 modules from tb3.py
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

        # pull in some useful publisher/subscriber functions from the tb3.py module:
        self.vel_controller = Tb3Move()
        self.tb3_odom = Tb3Odometry()
        self.tb3_lidar = Tb3LaserScan()

        rospy.loginfo("The 'Search Action Server' is active...")

    # The action's "callback function":
    def action_server_launcher(self, goal: SearchGoal):
        rate = rospy.Rate(10)

        success = True
        vel = goal.fwd_velocity #m/s
        dist = goal.approach_distance #m
        if vel > 0.26 or vel < 0:
            print("velocity out of range")
            success = False

        if dist > 0.26: # extend range?
            print("invalid distance")
            success = False

        if not success:
            self.result.total_distance_travelled = -1.0
            self.result.closest_object_angle = -1.0
            self.closest_object_distance = -1.0
            #self.result.image_path = "None [ABORTED]"
            self.actionserver.set_aborted(self.result)
            return

        print(f"Search goal recieved: fwd_vel = {vel} m/s, approach_distance = {dist} m.")

        # Get the robot's current odometry from the Tb3Odometry() class:
        self.posx0 = self.tb3_odom.posx
        self.posy0 = self.tb3_odom.posy
        # Get information about objects up ahead from the Tb3LaserScan() class:
        self.closest_object = self.tb3_lidar.min_distance
        self.closest_object_location = self.tb3_lidar.closest_object_position

        self.vel_controller.set_move_cmd(linear = vel, angular = 0.0)
 
        while self.tb3_lidar.min_distance > dist:
            # update LaserScan data:
            self.closest_object = self.tb3_lidar.min_distance
            self.closest_object_location = self.tb3_lidar.closest_object_position
 
            self.vel_controller.publish()

            # determine how far the robot has travelled so far:
            self.distance = sqrt(pow(self.posx0 - self.tb3_odom.posx, 2) + pow(self.posy0 - self.tb3_odom.posy, 2))

            self.result.total_distance_travelled = self.distance
            self.result.closest_object_angle = self.closest_object_location
            self.closest_object_distance = self.closest_object

            # check if there has been a request to cancel the action mid-way through:
            if self.actionserver.is_preempt_requested():
                print("action cancelled")
                self.actionserver.set_preempted(self.result)
                self.vel_controller.stop()
                success = False
                # exit the loop:
                break

            self.feedback.current_distance_travelled = self.distance
            self.actionserver.publish_feedback(self.feedback)

            rate.sleep()

        if success:
            rospy.loginfo("approach completed successfully.")
            self.vel_controller.stop()
            self.actionserver.set_succeeded(self.result)





if __name__ == '__main__':
    rospy.init_node("search_action_server")
    SearchActionServer()
    rospy.spin()