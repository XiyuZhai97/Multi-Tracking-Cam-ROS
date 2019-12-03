#!/usr/bin/env python
# Xiyu
import  sys
import  tty, termios
import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    try:
    #   rospy.spin()
        rospy.init_node('hello', anonymous=True)
        rate = rospy.Rate(60) # 10hz
        
        # if not rospy.is_shutdown():
        print("c")
    except rospy.ROSInterruptException:
        print("Stopping keyboard_publisher")