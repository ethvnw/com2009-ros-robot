#!/usr/bin/env python3

import roslaunch
import rospy
from pathlib import Path

path = Path.home().joinpath("catkin_ws/src/com2009_team5/maps/task_5")
path.mkdir(parents=True, exist_ok=True)
rospy.init_node("map_getter", anonymous=True)
rate = rospy.Rate(0.01)

launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

while not rospy.is_shutdown():    
    rospy.loginfo(f"Saving map at time: {rospy.get_time()}...")


    node = roslaunch.core.Node(package="map_server",
                            node_type="map_saver",
                            args=f"-f {path}")

    process = launch.launch(node)
    rate.sleep()
