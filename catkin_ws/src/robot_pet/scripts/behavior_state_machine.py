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
    def get_new_command(self):
        self._new_command_available = False
        return self._command



class GetPositionClient():
    def __init__(self):
        rospy.loginfo("Wait for service get_position to be available")
        rospy.wait_for_service('get_position')

    def call_srv(self, obj): 

        try:
            get_position = rospy.ServiceProxy('get_position', GetPosition)

            #Fill the request
            request = GetPositionRequest()

            request.object = obj
            
            rospy.loginfo("Requesting Position of {}".format(request.object))
            res = get_position(request)
            if res.success == True:
                rospy.loginfo("Position of {} returned: x={} y={}".format(request.object, res.point.x, res.point.y))
                return res.point.x, res.point.y
            else:
                rospy.loginfo("Position of {} not available!".format(request.object))
                return None, None

        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s"%e)



class SetTargetActionClient():
    def __init__(self):
        self.ready_for_new_target = True
        self.client = actionlib.SimpleActionClient('set_target_position_as', SetTargetPositionAction)
        rospy.loginfo("Waiting for action server to come up...")
        self.client.wait_for_server()

    def call_action(self, x, y): #TODO add parameter!
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



class SleepingTimer():
    def __init__(self):
        self.sleeping_time_range = (10, 15)  #Sleep between 10 and 15 seconds
        self.awake_time_range = (20, 30)  #Be awake for ...
        self.time_to_sleep = False
        self.timer = rospy.Timer(rospy.Duration(random.uniform(*self.awake_time_range)), self.callback, oneshot=True)

    def callback(self, msg):
        self.time_to_sleep = not self.time_to_sleep

        if self.time_to_sleep:
            rospy.loginfo("It's time to go to bed!")
            self.timer = rospy.Timer(rospy.Duration(random.uniform(*self.sleeping_time_range)), self.callback, oneshot=True)
        else:
            rospy.loginfo("It's time to wake up!")
            self.timer = rospy.Timer(rospy.Duration(random.uniform(*self.awake_time_range)), self.callback, oneshot=True)



########################################################
## STATE MACHINE CODE
#######################################################

# define state Foo
class Normal(smach.State):
    def __init__(self, get_position_client, pet_command_server, set_target_action_client, sleeping_timer):
        smach.State.__init__(self, outcomes=['cmd_play','sleeping_time'])
        
        self.get_position_client = get_position_client
        self.pet_command_server = pet_command_server
        self.set_target_action_client = set_target_action_client
        self.sleeping_timer = sleeping_timer

        self.map_width = rospy.get_param("/map_width")
        self.map_height = rospy.get_param("/map_height")
        

    def execute(self, userdata):
        rospy.loginfo('--- ENTERING STATE NORMAL ---')
        rate = rospy.Rate(10)
        while True:
            # Check if its time to sleep
            if self.sleeping_timer.time_to_sleep:
                return 'sleeping_time'

            # React to user commands
            if self.pet_command_server.is_new_command_available():
                cmd = self.pet_command_server.get_new_command()
                if cmd.command == 'play':
                    return 'cmd_play'
                if cmd.command =='go_to':
                    rospy.loginfo("Invalid command 'go_to' for state NORMAL. First say 'play' and then give go_to commands!")

            # Normal behavior: get random targets
            if self.set_target_action_client.ready_for_new_target:
                next_x = random.randint(0,self.map_width-1)
                next_y = random.randint(0,self.map_height-1) 
                self.set_target_action_client.call_action(next_x, next_y)
            
            rate.sleep()




# define state Sleep
class Sleep(smach.State):
    def __init__(self, get_position_client, pet_command_server, set_target_action_client, sleeping_timer):
        smach.State.__init__(self, outcomes=['slept_enough'])

        self.get_position_client = get_position_client
        self.pet_command_server = pet_command_server
        self.set_target_action_client = set_target_action_client
        self.sleeping_timer = sleeping_timer

    def execute(self, userdata):
        rospy.loginfo('--- ENTERING STATE SLEEP ---')
        rate = rospy.Rate(10)
        
        # Robot might still be moving => wait until last target reached
        while self.set_target_action_client.ready_for_new_target:
            rate.sleep()
        
        #Get position of house
        x,y = get_position_client.call_srv("house")
        
        #Set new target: house
        set_target_action_client.call_action(x,y)

        #If target reached: just wait until wake-up flag is set
        while True:
            # Ignore user commands -> Get the commands to consider command as hadeled
            if self.pet_command_server.is_new_command_available():
                cmd = self.pet_command_server.get_new_command()
                rospy.loginfo("Command is ignored, because Robot is sleeping")

            if not self.sleeping_timer.time_to_sleep:
                return 'slept_enough'
            rate.sleep()
            

# define state Play
class Play(smach.State):
    def __init__(self, get_position_client, pet_command_server, set_target_action_client, sleeping_timer):
        smach.State.__init__(self, outcomes=['played_enough','sleeping_time'])

        self.get_position_client = get_position_client
        self.pet_command_server = pet_command_server
        self.set_target_action_client = set_target_action_client
        self.sleeping_timer = sleeping_timer

    def execute(self, userdata):
        rospy.loginfo('--- ENTERING STATE PLAY ---')
        return 'played_enough'



if __name__ == "__main__":
    rospy.init_node('behavior_state_machine')

    
    get_position_client = GetPositionClient()
    pet_command_server = PetCommandServer()
    set_target_action_client = SetTargetActionClient()
    sleeping_timer = SleepingTimer()

    # Create a SMACH state machine
    sm_top = smach.StateMachine(outcomes=[])

    # Open the container
    with sm_top:
        # Add states to the container
        smach.StateMachine.add('NORMAL', Normal(get_position_client, pet_command_server, set_target_action_client, sleeping_timer), 
                               transitions={'cmd_play':'PLAY', 
                                            'sleeping_time':'SLEEP'})

        smach.StateMachine.add('SLEEP', Sleep(get_position_client, pet_command_server, set_target_action_client, sleeping_timer), 
                               transitions={'slept_enough':'NORMAL'})

        smach.StateMachine.add('PLAY', Play(get_position_client, pet_command_server, set_target_action_client, sleeping_timer), 
                               transitions={'played_enough':'NORMAL',
                                            'sleeping_time':'SLEEP' })

    # Execute SMACH plan
    outcome = sm_top.execute()

    # MOVE THIS INTO STATES!
    rospy.spin()