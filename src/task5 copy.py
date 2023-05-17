#! /usr/bin/env python3
import rospy
import actionlib
import tb3
import argparse
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pathlib import Path
from tuos_ros_msgs.msg import SearchAction, SearchGoal, SearchFeedback


# 0.2 and 3.5 is range lidar can see
# between that its 0, make sure it doesnt defualt to 0. 
class Task5():
    goal = SearchGoal()

    filters = [(-10, 150, 100),(10, 255, 255),(20,50,100),(40,255,255),
                (52,177,100),(75,255,255),(112,150,100),(128,255,255)]

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
        self.action_complete = False

        self.vel_controller = tb3.Tb3Move()
        self.tb3_lidar = tb3.Tb3LaserScan()
        self.distance = 0

        self.cam_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.cam_callback)
        self.cv = CvBridge()
        self.m00 = 0
        self.m00_min = 10000
        self.image_taken = False
        self.target_found = False
        self.ready_for_img = False

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

    def cam_callback(self, img):
        try:
            cv_img = self.cv.imgmsg_to_cv2(img, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        self.colours = {
            "blue":       [(115, 225, 100), (130, 255, 130)],
            "green":      [(20, 160, 100), (65, 255, 255)],
            "red":        [(0, 205, 100), (10, 255, 255)],
            "yellow":     [(25, 170, 100), (30, 255, 255)],
        }

        height, width, _ = cv_img.shape
        crop_width = width - 800
        crop_height = 400
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))

        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        self.hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        self.mask = cv2.inRange(self.hsv_img, self.colours[self.args.target_colour][0], self.colours[self.args.target_colour][1])
        m = cv2.moments(self.mask)
        self.m00 = m["m00"]
        self.cy = m['m10'] / (m['m00'] + 1e-5)
        self.cz = m['m01'] / (m['m00'] + 1e-5)

        if self.mask.any() and not self.image_taken:
            self.target_found = True

        if self.ready_for_img and not self.image_taken:
            self.image_taken = True
            path = Path.home().joinpath("catkin_ws/src/com2009_team5/snaps")
            path.mkdir(parents=True, exist_ok=True)

            cv2.imwrite(str(path.joinpath("the_beacon.jpg")), cv_img)
            rospy.loginfo("Image captured!")
           
        cv2.waitKey(1)

    def avoid_sides(self):
        threshold = 0.15
        side = ""
        dist = 0

        if self.tb3_lidar.min_left < threshold:
            self.vel_controller.set_move_cmd(angular=1.5)
            side = "left"
        elif self.tb3_lidar.min_right < threshold:
            self.vel_controller.set_move_cmd(angular=-1.5)
            side = "right"

        while dist < threshold:
            if side == "left":
                dist = self.tb3_lidar.min_left
            else:
                dist = self.tb3_lidar.min_right

            self.vel_controller.publish()
            self.rate.sleep()
            continue

        self.vel_controller.stop()


    def main_loop(self):
        self.goal.fwd_velocity = 0.2
        self.goal.approach_distance = 0.5

        while not rospy.is_shutdown():

            self.client.send_goal(self.goal, feedback_cb=self.feedback_callback)
            print("goal sent")
            while self.client.get_state() < 2:
                self.avoid_sides()

                if not self.image_taken and self.target_found:
                    self.client.cancel_goal()
                    rospy.loginfo("Target found!")
                    self.ready_for_img = True
                    rospy.sleep(3)  
                    break
                self.rate.sleep()

            print("goal ended")
      
            self.avoid_sides()


            self.rate.sleep()


if __name__ == '__main__':
    try:
        task5 = Task5()
        task5.main_loop()
    except rospy.ROSInterruptException:
        pass