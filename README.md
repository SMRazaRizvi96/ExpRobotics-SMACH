# ExpRobotics-SMACH
This repository contains the State Machine of a Robot

This is the 'Assignment 1: Behavioral Architecture' for the course 'Experimental Robotics'.

How to Run:

1. Give running permissions to it with
	$ chmod +x assignment.py

2. Install the visualizer using
   	$ sudo apt-get install ros-melodic-smach-viewer

3. Run the visualizer with
        $ rosrun smach_viewer smach_viewer.py

4. Run the launch file
	$ roslaunch smach_tutorial smachlaunch.launch

**************************

Files List:

1. Assignment.py: This is the State Machine
2. SpeakInteraction.cpp: This is a Node simulating the speech interaction of a person.
In this node, A User can type what the User wants to say. If the User types 'play', the parameter 'playflag' in the ROS Parameter server is set to 1.
	Otherwise, the 'playflag' parameter stays 0.

ROS Parameters Used:
1. 'playflag': This parameter is a flag which is 1 if the User has said 'play', and 0 otherwise.
2. 'velocity': This parameter controls the velocity of the system, since this parameter is used as an argument to the SLEEP command, to simulate the sleep time of the robot and as the time a robot takes to reach a particular position.


Implementation Details:

In this Assignment, we have developed a Finite State Machine for a Robot that has three states:

![alt text](https://github.com/SMRazaRizvi96/ExpRobotics-SMACH/blob/main/statemachine.png?raw=true)

1. NORMAL:
	In this state, the robot has to move on random locations.
	For this, the Robot first generates a random number between 2 and 5.
	This is the number of Random locations the Robot will go to.
Next, a FOR loop starting from 1 till this random number, is started to repeat the Robot movement.
	In every iteration of the FOR loop, first the 'playflag' parameter is checked.
	If the 'playflag' is 1, the outcome 'speech command:play' is returned and the Robot transitions to PLAY state.
	Otherwise, the Robot generates a random coordinate, takes some time to reach there, and stays there for some time.
	This is repeated in the FOR loop.
	Again, after the completion of the For loop, before transitioning to the SLEEP state,
	the 'playflag' is checked again.


2. SLEEP:
	In this state, the robot takes some time to reach a predefined Home coordinate and stays there for some time.
	After this, it returns 'after finishing sleep time' and Transitions to the NORMAL state.


3. PLAY:
	In this state, the Robot has to follow the User commands to go on specific locations given by the User.
	At first, a random number between 3 and 5 is generated. This is the number of times a User can give pointed gestures to the Robot.
	Next, a FOR loop starting from 1 till this random number, is started to repeat the Robot movement.
	First the robot comes near the User coordinate and waits for pointed gestures.
	Since our system is limited to give pointed gestures by typing the coordinates on the terminal,
	The User is first asked for the X coordinate, and then the Y coordinate.
	The Robot then takes some time to reach that coordinate, stays there, and again comes back near the User, and this repeat.
	After the FOR loop, the state returns 'after finishing play time' and the Robot transitions to the NORMAL state.
	

The Robot transitions the states on the following rules.

1. The Robot initially starts with the NORMAL state.
2. If the user types 'play' in the SpeakInteraction Node’s terminal, the Robot transitions to the PLAY state.
3. If the User doesn't type 'play', the Robot transitions to SLEEP state, from NORMAL state.
4. After SLEEP, again if the user hasn't said 'play', the Robot transitions to NORMAL state.

Limitations:

1. Since the System is not using any voice commands, one of the limitations is to type the command rather than saying it.
2. The System is also not using any cameras to capture the pointed location.
In order to command a Robot to go to a particular position while in the PLAY state,
the User has to write the coordinates of the location where the Robot should go.
3. The Robot goes into the PLAY state only when the user types 'play'. No other synonyms, or phrases are acceptable.
4. In the PLAY state, the user can only give 3 to 5 positions to the robot.
5. In the NORMAL state, the Robot only goes to 2 to 5 random locations only.

**************************

Authors:

Laiba Zahid (S4853477): S4853477@STUDENTI.UNIGE.IT
Syed Muhammad Raza Rizvi (S4853521): S4853521@STUDENTI.UNIGE.IT

Initially, we both developed two architectures.
The other architecture was such that rather than using a separate node and the 'playflag' ROS parameter to simulate the speech command,
The Robot while in the NORMAL state, used to ask on the terminal 'If you want to play with me, please type play'.
And if the ser types 'play', the state returns 'speech command:play' and transitions to PLAY state.
But after the discussion, we came to the conclusion that this Architecture has a lot of limitations,
and it doesn't replicate a real situation. So we both discussed and finalized the implemented architecture which is now closer
to the real scenario.
