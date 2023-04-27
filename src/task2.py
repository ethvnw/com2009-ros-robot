#! /usr/bin/env python3
# search_client.py

import rospy
import actionlib

from tuos_ros_msgs.msg import SearchAction, SearchGoal, SearchFeedback

class SearchActionClient():
    goal = SearchGoal()

    def feedback_callback(self, feedback_data: SearchFeedback):
        self.distance = feedback_data.current_distance_travelled
        print("distance traveled = {self.distance:.3f} m.")

    def __init__(self):
        self.distance = 0.0

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
            ## TODO: cancel the goal request, if this node is shutdown before the action has completed...



            rospy.logwarn("Goal Cancelled...")

        ## TODO: Print the result here...



    def main_loop(self):
        ## TODO: assign values to all goal parameters
        ## and send the goal to the action server...
        self.goal.approach_distance = 0.4 # m
        self.goal.fwd_velocity = 0.1 #m/s

        self.client.send_goal(self.goal,feedback_cb = self.feedback_callback)

        while self.client.get_state() < 2:
            ## TODO: Construct an if statement and cancel the goal if the 
            ## distance travelled exceeds 2 meters...
            if self.distance ...


                # break out of the while loop to stop the node:
                break

            self.rate.sleep()

        self.action_complete = True

if __name__ == '__main__':
    ## TODO: Instantiate the node and call the main_loop() method from it...
