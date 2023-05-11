#! /usr/bin/env python3
# search_client.py

import rospy
import actionlib
from tb3 import Tb3Move, Tb3LaserScan
from geometry_msgs.msg import Twist
from tuos_ros_msgs.msg import SearchAction, SearchGoal, SearchFeedback

# Import some other useful Python Modules
import random
from scipy.stats import levy
from math import sqrt, pow, radians

class Task2():
    def __init__(self):
        self.msg_counter = 0
        self.distance = 0.0
        self.action_complete = False
        rospy.init_node("Task2", anonymous = True)
        self.rate = rospy.Rate(1)
        
        self.client = actionlib.SimpleActionClient(
            "/com2009_team5_task2_server",
            SearchAction
        )
        self.client.wait_for_server()

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.goal = SearchGoal()
        self.scan = Tb3LaserScan()
        self.vel_controller = Tb3Move()

        rospy.on_shutdown(self.shutdown_ops)
        rospy.loginfo("The node has been initialised...")

    def shutdown_ops(self):
        if not self.action_complete:
            rospy.logwarn("Received a shutdown request. Cancelling Goal...")
            self.client.cancel_goal()
            rospy.logwarn("Goal Cancelled...")

        rospy.sleep(1)
        result = self.client.get_result()
        print(result)

        #self.pub.publish(Twist())

    def feedback_callback(self, feedback_data: SearchFeedback):
        self.distance = feedback_data.current_distance_travelled
        if self.msg_counter > 5:
            print(f"distance traveled = {self.distance:.3f} m.")
            self.msg_counter = 0
        else:
            self.msg_counter += 1

    def main_loop(self):
        self.goal.approach_distance = 0.2 # m
        self.goal.fwd_velocity = 0.2 # m/s

        while (not rospy.is_shutdown()): 
            self.action_complete = False
            print("Sending goal")
            self.client.send_goal(self.goal,feedback_cb = self.feedback_callback)

            while self.client.get_state() < 2:
                if self.distance > 2:
                    self.client.cancel_goal()
                    rospy.logwarn("Goal Cancelled...")
                    break
                self.rate.sleep()

            self.action_complete = True

            obj_angle = self.client.get_result().closest_object_angle
            if obj_angle > 0:
                self.vel_controller.set_move_cmd(angular=1.5)
            else:
                self.vel_controller.set_move_cmd(angular=-1.5)
            
            self.vel_controller.publish()

            print(f"Rotating with {self.vel_controller.vel_cmd.angular.z} angular velocity...")

            while (self.scan.min_distance < self.goal.approach_distance 
                and self.scan.min_left > 0.15 and self.scan.min_right > 0.15):
                continue

            # while self.scan.left_min > self.wall+0.1:
            #     self.client.cancel_goal()
            #     self.pub.publish(Twist())
            #     break

            #self.action_complete = True if self.client.get_state() == 3 else False


            # update LaserScan data:
            # levy_number = levy.rvs(scale=0.1,size=1)[0]

            # self.closest_object = self.tb3_lidar.min_distance
            # self.closest_object_location = self.tb3_lidar.closest_object_position

            # # #TODO: look into time going backwards error caused here vvv
            # # startTime = rospy.get_rostime()            

            # # while ((rospy.get_rostime().secs - startTime.secs) < levy_number) and (self.closest_object > dist):
            # #     self.update_odom()

            # #     #TODO: check if levy element acc works
            # #     self.vel_controller.set_move_cmd(vel,0)
            # # self.vel_controller.publish()

            self.vel_controller.stop()
            self.rate.sleep()


if __name__ == '__main__':
    node = Task2()
    try:
        node.main_loop()
    except rospy.ROSInterruptException:
        pass



