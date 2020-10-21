#!/usr/bin/env python
# basic publisher inserted
import rospy
import actionlib
from std_msgs.msg import String
from robot_pet.msg import *

def talker():
    pub = rospy.Publisher('pet_position', Point2d, queue_size=10)
    rospy.init_node('localizer_and_navigator', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    xpos = 2
    ypos = 3
    while not rospy.is_shutdown():
        msg = Point2d()
        msg.x = xpos
        msg.y = ypos
        rospy.loginfo("Publishing x={} y={}".format(msg.x, msg.y))
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass