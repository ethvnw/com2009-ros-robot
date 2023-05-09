#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import actionlib
from tuos_ros_msgs.msg import SearchAction, SearchGoal, SearchFeedback
import tb3

class Task3():

    def __init__(self):
        node_name = "Task3"
        rospy.init_node(node_name, anonymous=True)
        self.ctrl_c = False
        self.rate = rospy.Rate(1)

        self.client = actionlib.SimpleActionClient("/obstacle_avoidance_server", SearchAction)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.goal = SearchGoal()
        self.action_complete = False
        self.scan = tb3.Tb3LaserScan()


        rospy.on_shutdown(self.shutdownhook)
        rospy.loginfo(f"The {node_name} node has been initialised...")

    def shutdownhook(self):
        self.ctrl_c = True
        self.pub.publish(Twist())

    def feedback_callback(self, feedback):
        x = 0

    def main_loop(self):
        # self.wall = self.scan.left_min
        while not self.ctrl_c:
            self.rate.sleep()
            rospy.sleep(0.2)
            if not self.action_complete:
                print("Sending goal")
                #self.goal_sent = True
                self.goal.approach_distance = 0.4
                self.goal.fwd_velocity = 0.35
                self.client.send_goal(self.goal, feedback_cb=self.feedback_callback)
                self.action_complete = self.client.wait_for_result()

            # while self.scan.left_min > self.wall+0.1:
            #     self.client.cancel_goal()
            #self.pub.publish(Twist())

            self.rate.sleep()
            


if __name__ == "__main__":
    node = Task3()
    try:
        node.main_loop()
    except rospy.ROSInterruptException:
        pass