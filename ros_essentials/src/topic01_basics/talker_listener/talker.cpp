#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv)
{
	// Initiate new ROS node named "talker"
	ros::init(argc, argv, "talker_node");

	// Create a node handle: it is reference assigned to a new node
	ros::NodeHandle node;

	// Create a publisher with a topic "chatter" that will send a String message
	ros::Publisher chatter_publisher = node.advertise<std_msgs::String>("chatter", 1000);

	// Rate is a class to define frequency for a loop. Here is 1 message every 2s.
	ros::Rate loop_rate(0.5);

   int count = 0;
   while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C
   
        // Create a new String ROS message.
	    // Message definition in this link http://docs.ros.org/api/std_msgs/html/msg/String.html
	    std_msgs::String msg;

        // Create a string for the data
	    std::stringstream ss;
	    ss << "Hello World " << count;
	    // Assign the string data to ROS message data field
        msg.data = ss.str();

        // Print message content in the terminal
        ROS_INFO("[Talker] I published %s\n", msg.data.c_str());

        // Publish message
        chatter_publisher.publish(msg);

        // Call this function often for ROS to process incoming messages
        ros::spinOnce(); 

        // Sleep for the rest of the cycle, to enforce the loop rate
        loop_rate.sleep(); 

        count++;
   }
   return 0;
}
