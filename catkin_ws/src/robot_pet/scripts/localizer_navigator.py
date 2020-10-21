#!/usr/bin/env python
# basic publisher inserted
import rospy
import actionlib
import random
from robot_pet.msg import *

from robot_pet.msg import SetTargetPositionAction, SetTargetPositionActionFeedback, SetTargetPositionActionResult


class ActionServer():

    def __init__(self):
        self.a_server = actionlib.SimpleActionServer("set_target_position_as", SetTargetPositionAction, execute_cb=self.execute_cb, auto_start=False)
        self.pub = rospy.Publisher('pet_position', Point2d, queue_size=10)
        self.a_server.start()

    def execute_cb(self, goal):
        #Wait for a random time between 1 and 5 seconds 
        waiting_time = random.uniform(1, 5)
        rospy.sleep(waiting_time)
        #Server does not send intermediate feedback nor a result, just the flag that its done
        self.a_server.set_succeeded()

        msg = Point2d()
        msg.x = goal.target.x
        msg.y = goal.target.y
        rospy.loginfo("Publishing x={} y={}".format(msg.x, msg.y))
        self.pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('localizer_and_navigator')
    s = ActionServer()
    talker()