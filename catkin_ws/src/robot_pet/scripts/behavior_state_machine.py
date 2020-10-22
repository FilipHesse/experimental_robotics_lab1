#!/usr/bin/env python3

from __future__ import print_function

from robot_pet.srv import PetCommand, PetCommandResponse, PetCommandRequest, GetPosition, GetPositionRequest
from robot_pet.msg import SetTargetPositionAction, SetTargetPositionGoal, SetTargetPositionResult
import rospy
import actionlib
import smach
import random

class PetCommandServer:
    def __init__(self):
        rospy.Service('pet_command', PetCommand, self.handle_command)
        rospy.loginfo("Ready to receive commands.")
        self._new_command_available = False 
        self._command = PetCommandRequest()    #Make variables private, so that get_new_command() has to be used

    def handle_command(self, req):
        rospy.loginfo("Message received: {} {} {}".format(req.command, req.point.x, req.point.y))
        self._command = req
        self._new_command_available = True
        return PetCommandResponse()

    def is_new_command_available(self):
        return self._new_command_available

    # interface for state machine to check for new commands
    def get_new_command(self, req):
        self._new_command_available = False
        return self._command



class GetPositionClient():

    def __init__(self):
        pass

    def call_srv(self): #TODO add parameter!
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

class SetTargetActionClient():
    def __init__(self):
        self.ready_for_new_target = True
        self.client = actionlib.SimpleActionClient('set_target_position_as', SetTargetPositionAction)

    def call_action(self, x, y): #TODO add parameter!

        
        rospy.loginfo("Waiting for action server to come up...")
        self.client.wait_for_server()

        goal = SetTargetPositionGoal()
        self.ready_for_new_target = False
        goal.target.x = x
        goal.target.y = y
        self.client.send_goal(goal,
                        done_cb=self.callback_done)

        rospy.loginfo("Goal has been sent to the action server.")

    def callback_done(self, state, result):
        rospy.loginfo("Action server is done. State: %s" % (str(state)))
        self.ready_for_new_target = True

########################################################
## STATE MACHINE CODE
#######################################################

# define state Foo
class Normal(smach.State):
    def __init__(self, set_target_action_client, pet_command_server):
        smach.State.__init__(self, outcomes=['cmd_play','sleeping_time'])
        self.set_target_action_client = set_target_action_client
        self.pet_command_server = pet_command_server
        self.map_width = rospy.get_param("/map_width")
        self.map_height = rospy.get_param("/map_height")


    def execute(self, userdata):
        rospy.loginfo('--- ENTERING STATE NORMAL ---')
        rate = rospy.Rate(10)
        while True:
            if self.pet_command_server.is_new_command_available():
                #Process Command
                pass
            
            if self.set_target_action_client.ready_for_new_target:
                next_x = random.randint(0,self.map_width-1)
                next_y = random.randint(0,self.map_height-1) 
                self.set_target_action_client.call_action(next_x, next_y)
            
            # Dont do anything until Target position is reached
            if not self.set_target_action_client.ready_for_new_target:
                rate.sleep()




# define state Bar
class Sleep(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['slept_enough'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Sleep')
        #return 'outcome2'

# define state Bar
class Play(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['played_enough','sleeping_time'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Play')
        #return 'outcome2'



if __name__ == "__main__":
    rospy.init_node('behavior_state_machine')

    get_position_client = GetPositionClient()
    pet_command_server = PetCommandServer()
    set_target_action_client = SetTargetActionClient()

    # Call them without parameters for testing
    get_position_client.call_srv()
    #set_target_action_client.call_action()

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('NORMAL', Normal(set_target_action_client, pet_command_server), 
                               transitions={'cmd_play':'PLAY', 
                                            'sleeping_time':'SLEEP'})

        smach.StateMachine.add('SLEEP', Sleep(), 
                               transitions={'slept_enough':'NORMAL'})

        smach.StateMachine.add('PLAY', Play(), 
                               transitions={'played_enough':'NORMAL',
                                            'sleeping_time':'SLEEP' })

    # Execute SMACH plan
    outcome = sm.execute()

    # MOVE THIS INTO STATES!
    rospy.spin()