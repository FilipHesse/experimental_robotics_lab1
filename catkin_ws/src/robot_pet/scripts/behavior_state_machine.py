#!/usr/bin/env python3

from __future__ import print_function

from robot_pet.srv import PetCommand, PetCommandResponse, GetPosition, GetPositionRequest
from robot_pet.msg import SetTargetPositionAction, SetTargetPositionGoal, SetTargetPositionResult
import rospy
import actionlib

def handle_pet_command(req):
    rospy.loginfo("Message received: {} {} {}".format(req.command, req.point.x, req.point.y))
    resp = PetCommandResponse()
    resp.success = True
    resp.explanation = ""
    return resp

def pet_command_server():
    s = rospy.Service('pet_command', PetCommand, handle_pet_command)
    rospy.loginfo("Ready to receive commands.")

def get_position_client():
    rospy.loginfo("Wait for service get_position to be available")
    rospy.wait_for_service('get_position')
    object_list = ["user", "house", "pet", "banana"]
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

def set_target_client():
    client = actionlib.SimpleActionClient('set_target_position_as', SetTargetPositionAction)
    
    rospy.loginfo("Waiting for action server to come up...")
    client.wait_for_server()

    goal = SetTargetPositionGoal()
    goal.target.x = 2
    goal.target.y = 2
    client.send_goal(goal,
                    done_cb=callback_done)

    rospy.loginfo("Goal has been sent to the action server.")

def callback_done(state, result):
    rospy.loginfo("Action server is done. State: %s" % (str(state)))



if __name__ == "__main__":
    rospy.init_node('behavior_state_machine')
    get_position_client()
    pet_command_server()
    set_target_client()
    rospy.spin()