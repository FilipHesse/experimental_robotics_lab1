#!/usr/bin/env python

# Basic service client inserted
from __future__ import print_function

import sys
import rospy
from robot_pet.srv import *
import random

def pet_command_client():
    rospy.loginfo("Wait for service pet_command to be available")
    rospy.wait_for_service('pet_command')

    map_width = rospy.get_param("/map_width")
    map_height = rospy.get_param("/map_height")

    counter = 0
    while not rospy.is_shutdown():
        try:
            pet_command = rospy.ServiceProxy('pet_command', PetCommand)

            #Fill the request
            request = PetCommandRequest()
            request.header = rospy.Header()

            #Each fivth command is play, the other commands are go_to a random place within the range
            if (counter % 5) == 0:
                request.command = "play"
                request.point.x = 0 #Values for x and y don't matter
                request.point.y = 0 
            else:
                request.command = "go_to"
                request.point.x = random.randint(map_width)
                request.point.y = random.randint(map_height) 
            
            rospy.loginfo("Sending command: {} x={} y={}".format(request.command, request.point.x, request.point.y))
            res = pet_command(request)
            if res.success == True:
                rospy.loginfo("Command reached pet successfully")
            else:
                rospy.loginfo("Pet did not receive command: {}".format(res.explanation))
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s"%e)

        #incement counter
        counter += 1
        #Wait for a random time between 0.5 and 5 seconds 
        waiting_time = random.uniform(0.5, 5)
        rospy.sleep(waiting_time)

if __name__ == "__main__":
    rospy.init_node('ui')
    pet_command_client()