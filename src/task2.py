#! /usr/bin/env python3
# task2 .py

import rospy
import actionlib
from tb3 import Tb3Move, Tb3LaserScan
from geometry_msgs.msg import Twist
from tuos_ros_msgs.msg import SearchAction, SearchGoal, SearchFeedback

# Import some other useful Python Modules
import random
from scipy.stats import levy
from math import sqrt, pow, radians,log10, floor

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

    def turn_direction(self):
        obj = self.client.get_result()
        if obj != None:
            obj_angle = obj.closest_object_angle
            if obj_angle > 0:
                self.vel_controller.set_move_cmd(angular=1.5)
            else:
                self.vel_controller.set_move_cmd(angular=-1.5)
            
            self.vel_controller.publish()

    def random_turn(self):
        random_rotate = random.randint(0,180)
        current_rotate = 0

        self.vel_controller.set_move_cmd(0,0)
        self.vel_controller.publish()

        self.turn_direction()

        #print("turning")
        while (current_rotate < random_rotate):
            current_rotate += 1

        self.vel_controller.set_move_cmd(0,0)
        self.vel_controller.publish()

        self.vel_controller.set_move_cmd(self.goal.fwd_velocity,0)

        step_size = levy.rvs(scale=0.1,size=1)[0]
        step_inc = 10 ** (floor(log10(step_size)-1))
        current_step = 0.0   

    def main_loop(self):
        self.goal.approach_distance = 0.55 # m
        self.goal.fwd_velocity = 0.26 # m/s
        start_time = rospy.get_rostime()
        timeup = False
        
        while (not rospy.is_shutdown()): 
            self.action_complete = False
            self.client.send_goal(self.goal,feedback_cb = self.feedback_callback)

            while self.client.get_state() < 2:
                if self.distance > 2:
                    break
                self.rate.sleep()

            self.action_complete = True

            self.turn_direction()

            self.vel_controller.set_move_cmd(self.goal.fwd_velocity,0)

            #print(f"Rotating with {self.vel_controller.vel_cmd.angular.z} angular velocity...")
            
            step_size = levy.rvs(scale=0.1,size=1)[0]
            step_inc = 10 ** (floor(log10(step_size)-1))
            current_step = 0.0

            while (self.scan.min_distance < self.goal.approach_distance 
                and self.scan.min_left > 0.15 and self.scan.min_right > 0.15):
                # if ((rospy.get_rostime().secs - start_time.secs) < 90):
                self.random_turn()
                # else:
                #     self.client.cancel_goal()
                #     print("90 secs elapsed")
                #     timeup = True
                #     self.action_complete = True
                #     break

            print("outside")
            current_step += step_inc
            if (current_step>step_size):
                self.random_turn()
            self.rate.sleep()
        
            if timeup:
                self.vel_controller.stop()
                self.rate.sleep()
                break


if __name__ == '__main__':
    node = Task2()
    try:
        node.main_loop()
    except rospy.ROSInterruptException:
        pass



