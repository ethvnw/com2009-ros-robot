#! /usr/bin/env python3
# search_client.py

import rospy
import actionlib

from tuos_ros_msgs.msg import SearchAction, SearchGoal, SearchFeedback

class SearchActionClient():
    goal = SearchGoal()

    def feedback_callback(self, feedback_data: SearchFeedback):
        self.distance = feedback_data.current_distance_travelled
        if self.msg_counter > 5:
            print(f"distance traveled = {self.distance:.3f} m.")
            self.msg_counter = 0
        else:
            self.msg_counter += 1

    def __init__(self):
        self.distance = 0.0
        self.msg_counter = 0

        self.action_complete = False
        rospy.init_node("search_action_client")
        self.rate = rospy.Rate(1)

        self.client = actionlib.SimpleActionClient(
            "/com2009_team5_task2_server",
            SearchAction
        )
        self.client.wait_for_server()

        rospy.on_shutdown(self.shutdown_ops)

    def shutdown_ops(self):
        if not self.action_complete:
            rospy.logwarn("Received a shutdown request. Cancelling Goal...")
            self.client.cancel_goal()
            rospy.logwarn("Goal Cancelled...")

        rospy.sleep(1)
        result = self.client.get_result()
        print("result:")
        print(f" * Action state = {self.client.get_state()}")
        print(f" * total_distance_travelled = {result.total_distance_travelled:.3f} m")
        print(f" * closest_object_distance = {result.closest_object_distance:.3f} m")
        print(f" * closest_object_angle = {result.closest_object_angle:.1f} degrees")



    def main_loop(self):
        self.goal.approach_distance = 0.25 # m
        self.goal.fwd_velocity = 0.26 # m/s
        self.client.send_goal(self.goal,feedback_cb = self.feedback_callback)

        while self.client.get_state() < 2:
            if self.distance > 2:
                print("more than 2m - stopped")
                # break out of the while loop to stop the node:
                break

            self.rate.sleep()

        self.action_complete = True if self.client.get_state() == 3 else False

if __name__ == '__main__':
    none = SearchActionClient()
    none.main_loop()



