# ROS Basics

Author: __*yufanana*__

This documentation was produced from the ROS for Beginners: Basics, Motion and OpenCV course by Anis Koubaa available on udemy.com. :)
</br>
____

## Section 1: ROS Concepts

There are 3 communication patterns.

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

__roscore__ is the master node. It has to be started all the time.

ROS is like a network inside a local machine.

Then, start a new terminal window to run other ROS commands.

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
`rostopic list` to get a list of *topics* in a ROS computation graph. <br>

__Message Type__ <br>
`geometry_msgs/Twist` where
- `geometry_msgs` is the ROS package where the ROS message is located.
- `Twist` is the ROS message.
- So, `<package>/<message>`

`rosmsg show geometry_msgs/Twist` shows the message content.

ROS messages have *.msg* file types. <br> <br>


__Publish a message on a topic using CMD line__ <br>
`rostopic pub -r 10 /turtle1/cmd_vel geometry_msgs/Twist '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'` where
- `pub` means publish.
- `-r 10` means repeat 10 times.
- `/turtle1/cmd_vel` is the topic.
- `geometry_msgs/Twist` is the message type.
- `'{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z :0.0}}'` is the data in json format.

`rostopic echo /turtle1/cmd_vel` outputs the content of the messages in `cmd_vel` topic when the message is published.


This robot is only able to move using linear.x (forward, backward) and angular.z (rotate). This is sufficient for 2D motion.