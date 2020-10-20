#!/usr/bin/env python

# Basic service client inserted
from __future__ import print_function

import sys
import rospy
from robot_pet.srv import *
import random

def pet_command_client():
    rospy.wait_for_service('pet_command')

    x_min = -100
    x_max = 100

    y_min = -100
    y_max = 100

    counter = 0
    while not rospy.is_shutdown():

        #Each fivth command is play, the other commands are go_to a random place within the range
        if (counter % 5) == 0:
            msg = ("play",0,0)  #Values for x and y don't matter
        else:
            msg = ("go_to",random.randint(x_min,x_max),random.randint(y_min, y_max))
        try:
            pet_command = rospy.ServiceProxy('pet_command', PetCommand)
            success, answer_msg = pet_command(msg)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        #incement counter
        counter += 1
        #Wait for a random time between 0.5 and 5 seconds 
        waiting_time = random.uniform(0.5, 5)
        rospy.sleep(waiting_time)

if __name__ == "__main__":
    rospy.init_node('ui')
    pet_command_client()