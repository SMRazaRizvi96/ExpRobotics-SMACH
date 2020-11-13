#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import time
import random

# INSTALLATION
# - create ROS package in your workspace:
#          $ catkin_create_pkg smach_tutorial std_msgs rospy
# - move this file to the 'smach_tutorial/scr' folder and give running permissions to it with
#          $ chmod +x state_machine.py
# - run the 'roscore' and then you can run the state machine with
#          $ rosrun smach_tutorial assignment.py
# - install the visualiser using
#          $ sudo apt-get install ros-melodic-smach-viewer
# - run the visualiser with
#          $ rosrun smach_viewer smach_viewer.py


"""@package docstring
This is Finite State Machine which simulates the 3 States of a Robot

1. Normal
2. Play
3. Sleep

"""


"""
Variables defining locations in the space
"""

home_x= 1
home_y= 1
robot_x= home_x
robot_y= home_y
person_x= 25
person_y= 25

from std_msgs.msg import String

# define state Play
class Play(smach.State):
    def __init__(self):
	"""
	The Play state has only one outcome 'after finishing play time', that is 	 returned after finishing the play state, and transitions to the 'NORMAL' 	 state
	"""
        # initialisation function, it should not wait
	smach.State.__init__(self, 
                             outcomes=['after finishing play time'])
       
    def execute(self, userdata):
        """
        In the 'play' state, first the robot goes near the person.
        For this, it goes at the coordinate just before the person
        (-1 in both the x and y axis).
        After reaching near the person, it waits for a pointing Gesture.
        In this simulation, the pointing Gesture is simulated as a
        direct coordinated received as input.
        We are receiving the x and y coordinates separately.
        After receiving the coordinates, the robot reaches that position
        after some time.
        """ 
        
        rospy.loginfo('Executing state PLAY')


	i = random.randint(3,5)	
	for x in range(i):
		robot_x = person_x-1
		robot_y = person_y-1
		time.sleep(rospy.get_param("velocity"))
		print 'Reached Near Person:  (',robot_x,',',robot_y,')'
		print 'Please show with gesture where do you want me to go in x 		       coordinate:  ' 	
		gesture_x = raw_input()
		
		print 'Please show with gesture where do you want me to go in y 		       coordinate:  '
		gesture_y = raw_input()
		
		robot_x = int(gesture_x)
		robot_y = int(gesture_y)
		time.sleep(rospy.get_param("velocity"))
		print 'Reached at pointed position:  (',robot_x,',',robot_y,')'
	
        return 'after finishing play time'

# define state Sleep
class Sleep(smach.State):
    """

    This class simulates the 'Sleep' state of the Robot.
    The Robot will enter this state after some time on its own.
    In this state, the robot goes to the 'home' coordinate and sleeps for a while.

    """
    def __init__(self):

       """
       The Sleep state has only one outcome 'after finishing sleep time', that is returned after finishing the sleep state and transitions to the 'NORMAL' state.
       """
       # initialisation function, it should not wait
       smach.State.__init__(self, 
                             outcomes=['after finishing sleep time'])
        
    def execute(self, userdata):
        # function called when exiting from the node, it can be blacking
        rospy.loginfo('Executing state SLEEP')
	time.sleep(rospy.get_param("velocity"))
        robot_x= home_x;
	robot_y= home_y;
        print 'Reached Home:  (',robot_x,',',robot_y,')'
	print 'Sleeping...'
	time.sleep(rospy.get_param("velocity"))
        return 'after finishing sleep time'
    

# define state Normal
class Normal(smach.State):
    """

    This class simulates the 'NORMAL' state of the Robot.
    The robot goes into this state if the Robot is in Sleep state and the Person doesn't say 'play'.
    In this state, the robot goes on random locations.
    In our implementation, the number of locations the robot will go to, is a random variable calculated everytime the Robot enters this state.
    When the Robot is in the Normal state, it also keeps on checking the 'playflag' in the ROS Parameter server. If the parameter is 1, this means that the User has said 'play', and then the state transisitions to the PLAY state.

    """
    def __init__(self):
       """
       The Normal state has two outcomes.
       It returns 'speech command:play', if the Robot receives a play command.
       Otherwise, it returns 'if no user input for a certain time', to go into the SLEEP state.
       """
       smach.State.__init__(self, 
                             outcomes=['if no user input for a certain time',
 				       'speech command:play'],
			     input_keys=['normal_data_in'])

    def execute(self, userdata):
        while not rospy.is_shutdown():  
   
	    rospy.loginfo('Executing state NORMAL')
	    i = random.randint(2,5)
	    for x in range(i):
	    	checker = rospy.get_param("playflag")
	    	if (checker == 1):
	        	rospy.set_param("playflag", 0)
			return 'speech command:play'
	    	robot_x= random.randint(1,10)
	    	robot_y= random.randint(1,10)
	    	time.sleep(rospy.get_param("velocity"))
	    	print 'Reached Destination:  (',robot_x,',',robot_y,')'
	    checker = rospy.get_param("playflag")
	    if (checker == 1):
	        rospy.set_param("playflag", 0)
		return 'speech command:play'
	    return 'if no user input for a certain time'

        
def main():

    """
    This project works on two files.
    1. 'assignment.py' which is the State Machine.
    2. The 'SpeakerInteraction.cpp' node that simulates the user voice commands.
    The 'SpeakerInteraction.cpp' node turns the 'playflag' on the ROS Parameter serves to 1, as soon as the User types 'play' on that node.

    Initially the Robot Starts with the 'NORMAL' state and switches the state based on the User command, or on the randomness of the 'SLEEP' state.
    """
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])
    
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('NORMAL', Normal(), 
                               transitions={'if no user input for a certain time':'SLEEP',
					    'speech command:play': 'PLAY'},
			       remapping = {'normal_data_in':'play'})
        smach.StateMachine.add('SLEEP', Sleep(), 
                               transitions={'after finishing sleep time':'NORMAL'})
	smach.StateMachine.add('PLAY', Play(), 
                               transitions={'after finishing play time':'NORMAL'})


    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
