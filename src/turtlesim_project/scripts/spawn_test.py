#!/usr/bin/env python3

import rospy, roslaunch
from turtlesim.srv import Spawn
import random, os

current_directory = os.path.dirname(os.path.abspath(__file__))
os.chdir(current_directory)

launch_file_path = f"../launch/try1.launch"
    # da launch file el bey3mel run lel node el hatmove el turtle 
    # 34an at7akem fel turtle 3alatool awel mate3ml spawn

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