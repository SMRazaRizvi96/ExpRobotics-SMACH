#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <iostream>
#include <fstream>



using namespace std;

int main(int argc, char **argv)
{

/**
 * This is a node created to simulate the voice commands
 * being received from a Person.
 * The node is created in a way that only if a person
 * types 'play', the 'playflag' parameter in the ROS server
 * is set to 1, which can be later on used by others.
 */
  
  ros::init(argc, argv, "Speak");  
  ros::NodeHandle n;

  n.setParam("playflag", 0);  /*! The 'playflag' parameter is initally set to 0 */ 

  while (ros::ok())
  {
    std_msgs::String msg; 
    cout << "If you want to play with me? If yes, please say 'play' ";
    getline(cin, msg.data); 
	
    ROS_INFO("%s", msg.data.c_str());
    if(msg.data == "play")
    {
    	n.setParam("playflag", 1); /*!The 'playflag' parameter is set to 1 only when the condition is true*/ 

    }
    ros::spinOnce();
  }

  return 0;
}
