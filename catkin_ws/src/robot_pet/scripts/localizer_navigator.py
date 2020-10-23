#!/usr/bin/env python3
"""Simple ROS-node, that localizes robot and navigates to next target point

This script creates a ROS node, which simulates the robot naviagation. It
implements an action server, that can receive a new target position. The
server simply waits for some time and considers the position to be reached.
Then the positions is published to the topic pet_position.

The programmer has chosen an action server instead of a service, because the
action server does not block the client. This way the client can continue
working while the result of this action is computed.
"""

import rospy
import actionlib
import random
from robot_pet.msg import SetTargetPositionAction, SetTargetPositionActionFeedback, SetTargetPositionActionResult, SetTargetPositionResult, Point2d


class ActionServer():
    """Action server that receives new target and publishes reached position

    Attributes: 
    a_server (actionlib.SimpleActionServer): Action server for new targets
    pub (rospy.Publisher): Publisher to publish reached target
    """
    def __init__(self):
        """Initializes Attributes
        """
        self.a_server = actionlib.SimpleActionServer("set_target_position_as", SetTargetPositionAction, execute_cb=self.execute_cb, auto_start=False)
        self.pub = rospy.Publisher('pet_position', Point2d, queue_size=10)
        self.a_server.start()

    def execute_cb(self, goal):
        """Callback that gets executed when Action is called

        First sleeps for a random time (between 0.5 and 3 seconds) and then
        considers the target to be reached. This point is then published to the
        topic pet_position.

        Finally, the function returns the target point to the caller of the
        action, to inform the caller to be done.

        Args:
            goal (robot_pet/Point2d): New goal position
        """

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
    """Entry point of the script 

    Initializes note and instantiates Action server
    """
    rospy.init_node('localizer_and_navigator')
    s = ActionServer()