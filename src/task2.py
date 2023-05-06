#! /usr/bin/env python3
# search_client.py

import rospy
import actionlib
import tb3

from geometry_msgs.msg import Twist
from tuos_ros_msgs.msg import SearchAction, SearchGoal, SearchFeedback

class Task2():
    def __init__(self):
        self.distance = 0.0
        self.msg_counter = 0
        self.action_complete = False
        self.goal_sent = False
        self.ctrl_c = False
        
        rospy.init_node("Task2", anonymous = True)
        self.client = actionlib.SimpleActionClient(
            "/com2009_team5_task2_server",
            SearchAction
        )
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.goal = SearchGoal()
        self.scan = tb3.Tb3LaserScan()
        self.rate = rospy.Rate(1)

        rospy.on_shutdown(self.shutdown_ops)
        rospy.loginfo("The node has been initialised...")

        
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

        self.ctrl_c = True
        self.pub.publish(Twist())

    def feedback_callback(self, feedback_data: SearchFeedback):
        self.distance = feedback_data.current_distance_travelled
        if self.msg_counter > 5:
            print(f"distance traveled = {self.distance:.3f} m.")
            self.msg_counter = 0
        else:
            self.msg_counter += 1

    def main_loop(self):
        self.wall = self.scan.left_min

        while not self.ctrl_c:
            if not self.goal_sent:
                print("Sending goal")
                self.goal.approach_distance = 0.1 # m
                self.goal.fwd_velocity = 0.2 # m/s
                self.client.send_goal(self.goal,feedback_cb = self.feedback_callback)
                self.goal_sent = True

            while self.scan.left_min > self.wall+0.1:
                self.client.cancel_goal()
                self.pub.publish(Twist())
                break

            self.rate.sleep()
        self.action_complete = True if self.client.get_state() == 3 else False

if __name__ == '__main__':
    node = Task2()
    try:
        node.main_loop()
    except rospy.ROSInterruptException:
        pass



