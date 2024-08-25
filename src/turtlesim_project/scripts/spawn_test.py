#!/usr/bin/env python3

import rospy
from turtlesim.srv import Spawn
import random 
import roslaunch

# x is from 0 to 11
# y is from 0 to 11


def spawn_turtle():
    x = random.uniform(0, 11)
    y = random.uniform(0, 11)
    angle = random.uniform(0, 360)
    rospy.init_node('turtle_spawner', anonymous=True)

    rospy.wait_for_service('/spawn')

    try:
        spawn_turtle = rospy.ServiceProxy('/spawn', Spawn)
        response = spawn_turtle(x, y, angle, '')
        
        rospy.loginfo(f"Spawned a turtle named {response.name}")

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

    launch_file_path = '/home/mohamed_hani/catkin_ws/src/turtlesim_project/launch/try1.launch'
    # da launch file el bey3mel run lel node el hatmove el turtle 

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file_path])

    launch.start()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down.")
    finally:
        launch.shutdown()

if __name__ == '__main__':
    spawn_turtle()