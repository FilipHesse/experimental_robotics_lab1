#!/usr/bin/env python3
"""User localizer

This script creates a ROS node, which simulates a user moving in the
environment. The user slowly moves in the environment.

    Requirements:
        The following parameters need to be set in the ros parameter server:
            /map_width
            /map_height
            /user_pos_x
            /user_pos_y
        You can use the launchfile params.launch to set these to some 
        default values
"""

import rospy
from std_msgs.msg import String
from robot_pet.msg import *
import random
import numpy as np


def position_publisher():
    """Implements the movement of the user and publishes its position

    The initial point and the map dimensions are loaded from the ros parameter
    server. Then (with a frequency of 0.25 Hz) a position is published which is
    adjecent to the previous one (random direction, diagonal is also possible).
    """
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
    """Entry Point of this script calls publisher
    """
    try:
        position_publisher()
    except rospy.ROSInterruptException:
        pass