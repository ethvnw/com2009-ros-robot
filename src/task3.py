#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import actionlib
from tuos_ros_msgs.msg import SearchAction, SearchGoal, SearchFeedback
import tb3

class Task3_2():

    def __init__(self):
        node_name = "Task3_2"
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

    def server_callback(self, feedback: SearchFeedback):
        self.distance = feedback.current_distance_travelled
        #print(f"Distance travelled: {self.distance}")

    def main_loop(self):
        while not self.ctrl_c:
            self.rate.sleep()
            rospy.sleep(0.2)
            if not self.action_complete:
                print("Sending goal")
                self.goal.approach_distance = 0.45
                self.goal.fwd_velocity = 0.25
                self.client.send_goal(self.goal, feedback_cb=self.server_callback)
                self.action_complete = self.client.wait_for_result()
            
            self.rate.sleep()

if __name__ == "__main__":
    node = Task3_2()
    try:
        node.main_loop()
    except rospy.ROSInterruptException:
        pass

