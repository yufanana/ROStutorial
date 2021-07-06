# ROS Basics

Author: __*yufanana*__

This documentation was produced from the ROS for Beginners: Basics, Motion and OpenCV course by Anis Koubaa available on udemy.com.
</br>
____

## Section 1: ROS Concepts

There are 3 communication patterns. <br>
TCPROS layers are established between the nodes with the help of the master node before communication can begin.

### 1.1 Publisher/Subscriber
Publisher ->  (topic) -> Subscriber </br>
It supports 1:N, N:1, N:N </br>
Possible topics: location (x, y, theta), obstacle (x, y), temperature, pressure.

E.g. multiple nodes require images from a camera.

### 1.2 ROS Services
Client -> (service request) -> Server </br>
Server -> (responds to service) -> Client </br>
This is synchronous.

### 1.3 ROS ActionLib
Client -> (action goal) -> Server </br>
Server -> (action feedback) -> Client (while waiting) </br>
Server -> (action result) -> Client (when done) </br>
This is asynchronous, clients can do other stuff while waiting.

E.g. move base, navigation step.

### 1.4 ROS Computation Graph
ROS is composed of different nodes. Nodes can communicate with other nodes via messages (topics, services, actions, parameters). 

Each node can be a 
- publisher/subscriber
- service server/service client
- action server/action client

__roscore__ is the master node. It has to be started all the time. ROS is like a network inside a local machine. If the master node crashes, the other nodes may still be able to continue communicating.

Start a new terminal window to run other ROS commands.

`rosrun rqt_graph rqt_graph` displays the ROS computation graph.

### 1.5 Limitations of ROS
Not able to control a swarm of robots. (?)</br>
Not real-time, where all processes have the same priority. </br>
It requires a reliable network (bandwidth). </br>
It has a possible single point of failure. </br>

### 1.6 Benefits of ROS
Gives the user the ability to control the state of the robot, and the ability to read the state of the robot anytime.

## Section 2: Set Up
### 2.1 Installation
Follow the instructions on [ROS installation wiki](http://wiki.ros.org/ROS/Installation_). </br>
Look out for the duration of long-term support.

### 2.2 Workspace
Follow the instructions on [ROS workspace wiki](https://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment). </br>
Enter the command `gedit .bashrc` <br>
Then, ensure that `source /opt/ros/noetic/setup.bash` is already included near the end.

To activate the catkin workspace, you will need to source/access the `setup.bash` file. </br>
To do so, enter the command `gedit .bashrc` then add `source /home/yufanana/catkin_ws/devel/setup.bash` into the file, where `yufanana` should be changed to your path.

Next, `source .bashrc` to active catkin workspace as default workspace.



## Section 3: *turtlesim*

After entering a keyword, *double tab* to view all the possible commands.

### 3.1 ROS Nodes
`roscore` to start the master node. <br>
`rosnode list` to get a list of nodes in a ROS computation graph. <br>
`rosrun turtlesim turtlesim_node` where<br>
- `rosrun` to run a node. <br>
- `turtlesim` is the ROS package where the ROS node is located. <br>
- `turtlesim_node` is the node to execute.
- So, `rosrun <package> <node>` to execute a node.

`rosrun turtlesim turtle_teleop_key` corresponds to the keyboard. It publishes info and sends messages to the `turtlesim_node`.

`rosnode info /teleop_turtle` gives information about the `teleop_turtle` node, e.g.
- Publications
- Subscriptoins
- Services
- Connections

### 3.2 Computation Graph
turtle_teleop_key <--> master node <--> turtlesim_node <br>
turtle_teleop_key -> turtlesim_node, where the topic of the messages is */turtle1/cmd_vel*.

`rosrun rqt_graph rqt_graph` displays the ROS computation graph.

### 3.3 ROS Message
__Topic__
`rostopic list` to get a list of *topics* in a ROS computation graph. <br>
e.g. 
- /turtle1/cmd_vel
- /turtle1/color_sensor
- /turtle1/pose

__Type__ <br>
e.g.
- turtlesim/Pose (for /turtle1/pose)
- geometry_msgs/Twist )for /turtle1/cmd_vel) 

`geometry_msgs/Twist` where
- `geometry_msgs` is the ROS package where the ROS message is located.
- `Twist` is the ROS message.
- So, `<package>/<message>`

`rosmsg show geometry_msgs/Twist` shows the message content.

ROS messages have *.msg* file types. <br> <br>

__Content__
e.g. turtlesim/Pose
'''
float32 x
float32 y
float32 theta
float32 linear_velocity
float32 angular_velocity
'''

__Publish a message on a topic using CMD line__ <br>
`rostopic pub -r 10 /turtle1/cmd_vel geometry_msgs/Twist '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'` where
- `pub` means publish.
- `-r 10` means repeat 10 times.
- `/turtle1/cmd_vel` is the topic.
- `geometry_msgs/Twist` is the message type.
- `'{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z :0.0}}'` is the data in json format.

`rostopic echo /turtle1/cmd_vel` outputs the content of the messages in `cmd_vel` topic when the message is published.


This robot is only able to move using linear.x (forward, backward) and angular.z (rotate). This is sufficient for 2D motion.

### Write Publisher of ROS Topics
1. Determine a name for topic.
2. Determine the type of the messages that the topic will be publish.
3. Determine the frequency of topic publication (per second).
4. Create a publisher object with the above parameters.
5. Keep publishing the topic message at the selected frequency.

Python
```python
import rospy
from std_msgs.msg import String
# std_msgs.msg is the package

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    # Creates publisher object
    # 'chatter' is topic, String is topic type, queue_size is like a buffer/queue

    rospy.init_node('talker', anonymous = True)
    # Initialise rosnode
    # 'talker' is name of the node, anonymous = True ensures that nodes have unique names/ID

    rate = rospy.Rate(1) # in Hz

    i = 0       # counter
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % i
        rospy.loginfo(hello_str)
        # Outputs into terminal
        pub.publish(hello_str)
        rate.sleep()        # sleep duration(s) = 1/rate
        i += 1
```

C++         *Message type is not defined in node instantiation, but in the callback function.*
```c++
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
```

### Write Subscriber to ROS Topics
1. Identify the name for the topic to listen to.
2. Identify the type of the messages to be received.
3. Define a *callback function* that will be executed when a new message is received.
4. Start listening for the topic messages.
5. Spin to listen forever (in C++).

Python
```python
import rospy
from std_msgs.msg import String

def chatter_callback(message):
    rospy.loginto(rospy.get_caller_id() + "I heard %s", message.data)
    # Outputs into termial

    # print("I heard %s", message.data)

def listener():
    rospy.init_node('listener', anonymous = True)

    rospy.Subscriber("chatter", String, chatter_callback)
    # Creates subscriber object
    # chatter_callback is the callback function

    rospy.spin()
    # Start listening
```

C++
```c++
#include "ros/ros.h"
#include "std_msgs/String.h"

// Topic messages callback
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("[Listener] I heard: [%s]\n", msg->data.c_str());
}

int main(int argc, char **argv)
{
    // Initiate a new ROS node named "listener"
	ros::init(argc, argv, "listener_node");
	//create a node handle: it is reference assigned to a new node
	ros::NodeHandle node;

    // Subscribe to a given topic, in this case "chatter".
	//chatterCallback: is the name of the callback function that will be executed each time a message is received.
    ros::Subscriber sub = node.subscribe("chatter", 1000, chatterCallback);

    // Enter a loop, pumping callbacks
    ros::spin();

    return 0;
}
```

### CMakeLists.txt
File that provides all the information for the C compiler to compile and execute. <br>
Define all the dependencies and packages used.

```txt
cmake_minimum_required(VERSION 2.8.3)
project(ros_essentials_cpp)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
)

# talker, creates .exe file called talker_node.exe
add_executable(talker_node src/topic01_basics/talker_listenenr/talker.cpp)
target_link_libraries(talker_node ${catkin_LIBRARIES})

# listener, creates .exe file called listener_node.exe
add_executable(listener_node src/topic01_basics/talker_listenenr/listener.cpp)
target_link_libraries(listener_node ${catkin_LIBRARIES})
```

### package.xml
Used by `catkin_make`
```xml
<buildtool_depend>catkin</buildtool_depend>

<build_depend>roscpp</build_depend>
<build_depend>rospy</build_depend>
<build_depend>std_msgs</build_depend>

<build_export_depend>roscpp</build_export_depend>
<build_export_depend>rospy</build_export_depend>
<build_export_depend>std_msgs</build_export_depend>

<exec_depend>roscpp</exec_depend>
<exec_depend>rospy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```