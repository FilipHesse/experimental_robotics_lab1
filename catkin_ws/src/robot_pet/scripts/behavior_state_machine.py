#!/usr/bin/env python3

from __future__ import print_function

from robot_pet.srv import PetCommand, PetCommandResponse
import rospy

def handle_pet_command(req):
    rospy.loginfo("Message received: {} {} {}".format(req.command, req.point.x, req.point.y))
    resp = PetCommandResponse()
    resp.success = True
    resp.explanation = ""
    return resp

def pet_command_server():
    s = rospy.Service('pet_command', PetCommand, handle_pet_command)
    rospy.loginfo("Ready to receive commands.")
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node('behavior_state_machine')
    pet_command_server()