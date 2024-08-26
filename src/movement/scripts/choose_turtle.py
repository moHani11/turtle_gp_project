#!/usr/bin/env python

import rospy
from turtlesim.srv import Spawn
from turtlesim.msg import Pose

def spawn_turtles():
    rospy.init_node('spawn_turtles', anonymous=True)
    rospy.wait_for_service('spawn')
    
    try:
        spawn_turtle = rospy.ServiceProxy('spawn', Spawn)
        
        # Coordinates for spawning turtles
        x_positions = [2.0, 2.0, 5.66, 9.32, 2.0, 5.66, 9.32]
        y_position = [2.0, 2.0, 2.0, 2.0, 9.32, 9.32, 9.32]  # y-position is constant to line up turtles horizontally
        angle = 0  # orientation angle (radians)
        
        # Spawn the additional turtles
        for i in range(1, len(x_positions)):
            name = 'turtle' + str(i + 1)  # Naming turtles as turtle2, turtle3, etc.
            spawn_turtle(x_positions[i], y_position[i], angle, name)
            rospy.loginfo(f'Spawned {name} at x={x_positions[i]}, y={y_position}[i]')
    
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    try:
        spawn_turtles()
    except rospy.ROSInterruptException:
        pass
