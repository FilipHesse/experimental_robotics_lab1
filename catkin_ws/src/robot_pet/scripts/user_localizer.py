#!/usr/bin/env python3
# basic publisher inserted
import rospy
from std_msgs.msg import String
from robot_pet.msg import *
import random
import numpy as np

def position_publisher():
    pub = rospy.Publisher('user_position', Point2d, queue_size=10)
    rospy.init_node('user_localizer')
    rate = rospy.Rate(0.25) # Hz

    map_width = rospy.get_param("/map_width")
    map_height = rospy.get_param("/map_height")
    
    msg = Point2d()
    msg.x = rospy.get_param("/user_pos_x")
    msg.y = rospy.get_param("/user_pos_y")
    rospy.logdebug("Publishing x={} y={}".format(msg.x, msg.y))
    pub.publish(msg)

    while not rospy.is_shutdown():
        next_x = msg.x + random.randint(-1,1)
        next_y = msg.x + random.randint(-1,1)

        # If next position is out of the map: repeat random 
        while(not next_x in np.arange(0,map_width) or not next_y in np.arange(0,map_height) ):
            next_x = msg.x + random.randint(-1,1)
            next_y = msg.y + random.randint(-1,1)

        msg.x = next_x
        msg.y = next_y
        rospy.logdebug("Publishing x={} y={}".format(msg.x, msg.y))
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        position_publisher()
    except rospy.ROSInterruptException:
        pass