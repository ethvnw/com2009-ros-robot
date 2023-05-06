#! /usr/bin/env python3
import rospy
import actionlib
import tb3
import argparse

from tuos_ros_msgs.msg import SearchAction, SearchGoal, SearchFeedback

class Task5():
    goal = SearchGoal()


    def __init__(self):
        rospy.init_node("Task5")
        self.rate = rospy.Rate(1)

        cli = argparse.ArgumentParser(description="Command-line interface for Task 5 node")
        cli.add_argument(
            "-target_colour",
            metavar="COL",
            default="red",
            choices=["red", "green", "blue", "yellow"],
            help="The colour of the target object to search for."
        )
        self.args = cli.parse_args(rospy.myargv()[1:])

        self.client = actionlib.SimpleActionClient("/task5_server", SearchAction)
        self.client.wait_for_server()

        self.vel_controller = tb3.Tb3Move()
        self.tb3_lidar = tb3.Tb3LaserScan()
        self.distance = 0

        rospy.loginfo("The 'Task5' node is active...")
        rospy.loginfo(f"TASK 5 BEACON: The target is {self.args.target_colour}.")
        rospy.on_shutdown(self.shutdown_ops)

    def shutdown_ops(self):
        self.vel_controller.stop()
        if not self.action_complete:
            rospy.logwarn("Received a shutdown request. Cancelling Goal...")
            self.client.cancel_goal()
            rospy.logwarn("Goal Cancelled...")

        print(self.client.get_result())

    def feedback_callback(self, feedback_data: SearchFeedback):
        self.distance = feedback_data.current_distance_travelled

    def main_loop(self):
        self.goal.fwd_velocity = 0.2
        self.goal.approach_distance = 0.45

        while not rospy.is_shutdown():
            self.action_complete = False
            self.client.send_goal(self.goal, feedback_cb=self.feedback_callback)
            
            while self.client.get_state() < 2:
                if self.distance > 2:
                    self.client.cancel_goal()
                    rospy.logwarn("Goal Cancelled... Distance exceeded 2 meters.")
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
           
            while (self.tb3_lidar.min_distance < self.goal.approach_distance 
                and self.tb3_lidar.min_left > 0.15 and self.tb3_lidar.min_right > 0.15):
                continue

            self.vel_controller.stop()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        task5 = Task5()
        task5.main_loop()
    except rospy.ROSInterruptException:
        pass