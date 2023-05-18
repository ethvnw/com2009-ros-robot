#! /usr/bin/env python3
import rospy
import actionlib
from tuos_ros_msgs.msg import SearchAction, SearchFeedback, SearchResult, SearchGoal
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan
from math import sqrt, pow

class ObstacleAvoidanceServer():
    feedback = SearchFeedback() 
    result = SearchResult()

    def __init__(self):
        self.node_name = "ObstacleAvoidanceServer"
        self.actionserver = actionlib.SimpleActionServer("/obstacle_avoidance_server", SearchAction, self.action_server_launcher, auto_start=False)
        self.vel_controller = Tb3Move()
        self.tb3_odom = Tb3Odometry()
        self.tb3_lidar = Tb3LaserScan()

        self.actionserver.start()
        rospy.loginfo(f"The {self.node_name} is active...")

    def action_server_launcher(self, goal: SearchGoal):
        rate = rospy.Rate(10)
        success = False
        print("Goal received: approach dist: %.2f" % goal.approach_distance + ", fwd vel: %.2f" % goal.fwd_velocity)
        
        self.posx0 = self.tb3_odom.posx
        self.posy0 = self.tb3_odom.posy
        self.prev_posx = self.posx0
        self.prev_posy = self.posy0
        self.update_odom()


        while not self.actionserver.is_preempt_requested():
            self.vel_controller.set_move_cmd(goal.fwd_velocity)
            self.vel_controller.publish()

            while self.closest_object > goal.approach_distance:
                self.update_odom()
                rate.sleep()

            self.vel_controller.stop()

            while self.closest_object < goal.approach_distance:
                self.update_odom()
                
                if self.tb3_lidar.min_left > self.tb3_lidar.min_right:
                    #print("moving left oa")
                    self.vel_controller.set_move_cmd(0, 0.7)
                else:
                    #print("moving right oa")
                    self.vel_controller.set_move_cmd(0, -0.7)
                self.vel_controller.publish()


            if success:
                rospy.loginfo("approach completed successfully.")
                self.vel_controller.stop()
                self.actionserver.set_succeeded(self.result)

        self.actionserver.set_preempted()
        self.vel_controller.stop()           
        success = False
        print("The action has been preempted.")


    def update_odom(self):
        self.feedback.current_distance_travelled += sqrt(pow(self.prev_posx - self.tb3_odom.posx, 2) + pow(self.prev_posy - self.tb3_odom.posy, 2))
        self.prev_posx = self.tb3_odom.posx
        self.prev_posy = self.tb3_odom.posy
    
        self.closest_object = self.tb3_lidar.min_distance
        self.closest_object_location = self.tb3_lidar.closest_object_position
        self.distance = sqrt(pow(self.posx0 - self.tb3_odom.posx, 2) + pow(self.posy0 - self.tb3_odom.posy, 2))


        self.result.total_distance_travelled = self.feedback.current_distance_travelled
        self.result.closest_object_distance = self.closest_object
        self.result.closest_object_angle = self.closest_object_location
        self.actionserver.publish_feedback(self.feedback)




if __name__ == '__main__':
    rospy.init_node("obstacle_avoidance_server")
    ObstacleAvoidanceServer()
    rospy.spin()
