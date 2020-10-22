#!/usr/bin/env python3

from __future__ import print_function

from robot_pet.srv import PetCommand, PetCommandResponse, GetPosition, GetPositionRequest
from robot_pet.msg import SetTargetPositionAction, SetTargetPositionGoal, SetTargetPositionResult
import rospy
import actionlib
import smach

class PetCommandServer:
    def __init__(self):
        s = rospy.Service('pet_command', PetCommand, self.handle_command)
        rospy.loginfo("Ready to receive commands.")

    def handle_command(self, req):
        rospy.loginfo("Message received: {} {} {}".format(req.command, req.point.x, req.point.y))
        resp = PetCommandResponse()
        resp.success = True
        resp.explanation = ""
        return resp



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
        pass

    def call_action(self): #TODO add parameter!
        client = actionlib.SimpleActionClient('set_target_position_as', SetTargetPositionAction)
        
        rospy.loginfo("Waiting for action server to come up...")
        client.wait_for_server()

        goal = SetTargetPositionGoal()
        goal.target.x = 2
        goal.target.y = 2
        client.send_goal(goal,
                        done_cb=self.callback_done)

        rospy.loginfo("Goal has been sent to the action server.")

    def callback_done(self, state, result):
        rospy.loginfo("Action server is done. State: %s" % (str(state)))

########################################################
## STATE MACHINE CODE
#######################################################

# define state Foo
class Normal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['cmd_play','sleeping_time'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state Normal')
        rospy.Rate(5)
        while True:
            pass



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
    set_target_action_client.call_action()

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('NORMAL', Normal(), 
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