#!/usr/bin/env python3

import rospy
from turtlesim.srv import Spawn
import random, os
import subprocess

current_directory = os.path.dirname(os.path.abspath(__file__))
os.chdir(current_directory)

launch_file_path = os.path.join(current_directory, "../launch/try1.launch")
    # da launch file el bey3mel run lel node el hatmove el turtle 
    # 34an at7akem fel turtle 3alatool awel mate3ml spawn

# x is from 0 to 11
# y is from 0 to 11


def spawn_turtle():
    x = random.uniform(0, 11)
    y = random.uniform(0, 11)
    angle = random.uniform(0, 360)  
    rospy.init_node('turtle_spawner', anonymous=True)  # bey3mel node  betspawn turtle fe makan random

    rospy.wait_for_service('/spawn')

    try:
        spawn_turtle = rospy.ServiceProxy('/spawn', Spawn)
        response = spawn_turtle(x, y, angle, '')
        
        rospy.loginfo(f"Spawned a turtle named: {response.name}")

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

    command = f"roslaunch turtlesim_project moveLaunch.launch turtle_name:={response.name}"
    # beysta5dem el bash command line 34an ycall el .launch file we yedeelo arguments

    subprocess.run(command, shell=True, capture_output=True, text=True)

if __name__ == '__main__':
    spawn_turtle()