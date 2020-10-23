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
        waiting_time = random.uniform(0.5, 3)
        rospy.sleep(waiting_time)


        msg = Point2d()
        msg.x = goal.target.x
        msg.y = goal.target.y

        res = SetTargetPositionResult()
        res.final_position.x = msg.x
        res.final_position.y = msg.y

        rospy.logdebug("Publishing x={} y={}".format(msg.x, msg.y))
        self.pub.publish(msg)

        #Server does not send intermediate feedback just a result and the flag that its done
        self.a_server.set_succeeded(res)


if __name__ == '__main__':
    rospy.init_node('localizer_and_navigator')
    s = ActionServer()