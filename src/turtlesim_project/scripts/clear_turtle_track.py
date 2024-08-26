#!/usr/bin/env python3
import rospy
from std_srvs.srv import Empty

def clear_track():
    rospy.init_node('clear_turtle_track', anonymous=True)
    rospy.wait_for_service('/clear')
    clear_service = rospy.ServiceProxy('/clear', Empty)

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        try:
            clear_service()
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
        rate.sleep()

if __name__ == "__main__":
    try:
        clear_track()
    except rospy.ROSInterruptException:
        pass
