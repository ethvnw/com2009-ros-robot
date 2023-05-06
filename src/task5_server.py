#! /usr/bin/env python3

import rospy
import actionlib
from tuos_ros_msgs.msg import SearchAction, SearchFeedback, SearchResult, SearchGoal
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan
from math import sqrt, pow

class Task5Server():
    feedback = SearchFeedback() 
    result = SearchResult()

    def __init__(self):
        self.actionserver = actionlib.SimpleActionServer(
            "/task5_server", SearchAction, self.action_server_launcher, auto_start=False)
        self.actionserver.start()

        self.vel_controller = Tb3Move()
        self.tb3_odom = Tb3Odometry()
        self.tb3_lidar = Tb3LaserScan()

        rospy.loginfo("The 'Task 5 Server' is active...")

    # The action's "callback function":
    def action_server_launcher(self, goal: SearchGoal):
        rate = rospy.Rate(10)

        success = True
        if goal.fwd_velocity <= 0 or goal.fwd_velocity > 0.26:
            print("Invalid velocity! Select a value between 0 and 0.26 m/s.")
            success = False

        if not success:
            self.actionserver.set_aborted(self.result)
            return

        print(f"Goal received:\n"
            f"Velocity: {goal.fwd_velocity:.2} m/s\n"
            f"Approach distance: {goal.approach_distance:.2} m\n")
        
        # Get the robot's current odometry from the Tb3Odometry() class:
        self.posx0 = self.tb3_odom.posx
        self.posy0 = self.tb3_odom.posy
        # Get information about objects up ahead from the Tb3LaserScan() class:
        self.closest_object = self.tb3_lidar.min_distance
        self.closest_object_location = self.tb3_lidar.closest_object_position

        self.vel_controller.set_move_cmd(goal.fwd_velocity)


        while self.closest_object > goal.approach_distance:
            self.closest_object = self.tb3_lidar.min_distance
            self.closest_object_location = self.tb3_lidar.closest_object_position

            self.vel_controller.publish()

            if self.actionserver.is_preempt_requested():
                rospy.loginfo("Cancelling the action.")
                self.actionserver.set_preempted(self.result)
                self.vel_controller.stop()
                success = False
                break

            self.distance = sqrt(pow(self.posx0 - self.tb3_odom.posx, 2) + pow(self.posy0 - self.tb3_odom.posy, 2))
            self.feedback.current_distance_travelled = self.distance
            self.actionserver.publish_feedback(self.feedback)

            self.result.total_distance_travelled = self.distance
            self.result.closest_object_distance = self.closest_object
            self.result.closest_object_angle = self.closest_object_location

            rate.sleep()

        self.vel_controller.stop()
        if success:
            rospy.loginfo("approach completed successfully.")
            self.actionserver.set_succeeded(self.result)


if __name__ == '__main__':
    rospy.init_node("search_action_server")
    Task5Server()
    rospy.spin()
