#!/usr/bin/env python3

from __future__ import print_function

from robot_pet.srv import PetCommand, PetCommandResponse, GetPosition, GetPositionRequest
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

def get_position_client():
    rospy.loginfo("Wait for service get_position to be available")
    rospy.wait_for_service('get_position')
    object_list = ["person", "house", "pet", "banana"]
    for obj in object_list:
        try:
            get_position = rospy.ServiceProxy('get_position', GetPosition)

            #Fill the request
            request = GetPositionRequest()

            request.object = obj
            
            rospy.loginfo("Requesting Position of {}".format(request.object))
            res = get_position(request)
            if res.success == True:
                rospy.loginfo("Position of {} returned: x={} y={}".format(request.object, res.point.x, res.point.y))
            else:
                rospy.loginfo("Position of {} not available!".format(request.object))
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s"%e)

        #Wait for a random time between 0.5 and 5 seconds 
        waiting_time = 1
        rospy.sleep(waiting_time)


if __name__ == "__main__":
    rospy.init_node('behavior_state_machine')
    get_position_client()
    pet_command_server()