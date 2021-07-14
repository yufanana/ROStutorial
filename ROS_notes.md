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

### 2.3 Packages
catkin packages should contain `package.xml` & `CMakeLists.txt` in its own folder. 

To create a new package, `cd catkin_ws/src` <br>
`catkin_create_pkg <package_name> <depend1> <depend2d> <depend3>` <br>
`catkin_create_pkg <package_name> std_msgs rospy ros cpp`

### 2.4 Launch File
Launches/rosrun multiple nodes instead of opening many terminals and typing in the command.

Can use `roslaunch package_name file_name.launch` to skip the roscore command.
```xml
<launch>
  <node name="turtlesim" pkg="turtlesim" type="turtlesim_node" />
  <node name="cleaner_node" pkg="ros_essentials" type="turtlesim_cleaner.py" />
</launch>
```

## Section 3: ROS Messages 

After entering a keyword, *double tab* to view all the possible commands.

### 3.1 ROS Nodes
`roscore` to start the master node. <br>
`rosnode list` to get a list of nodes in a ROS computation graph. <br>
`rosnode info /teleop_turtle` gives information about the `teleop_turtle` node, e.g.
- Publications
- Subscriptoins
- Services
- Connections
<br>

`rosrun <package_name> <py_file>` runs the python file. <br>
`rosrun turtlesim turtlesim_node` where<br>
- `rosrun` to run a node. <br>
- `turtlesim` is the ROS package where the ROS node is located. <br>
- `turtlesim_node` is the node to execute.
- So, `rosrun <package> <node>` to execute a node.

`rosrun turtlesim turtle_teleop_key` corresponds to the keyboard. It publishes info and sends messages to the `turtlesim_node`.



### 3.2 Computation Graph
turtle_teleop_key <--> master node <--> turtlesim_node <br>
turtle_teleop_key -> turtlesim_node, where the topic of the messages is */turtle1/cmd_vel*.

`rosrun rqt_graph rqt_graph` displays the ROS computation graph.

### 3.3 Message Definition
ROS messages have *.msg* file types. <br>

`rosmsg show <package>/<message_type>` shows the message content. <br>
e.g. `rosmsg show geometry_msgs/Twist` 

__Topic__ <br>
`rostopic list` to get a list of *topics* in a ROS computation graph. <br>
e.g. 
- /turtle1/cmd_vel
- /turtle1/color_sensor
- /turtle1/pose

__Type__ <br>
e.g. *(package_name/message_type)*
- turtlesim/Pose (for /turtle1/pose)
- geometry_msgs/Twist (for /turtle1/cmd_vel) 

__Content__ <br>
e.g. for turtlesim/Pose
```
float32 x
float32 y
float32 theta
float32 linear_velocity
float32 angular_velocity
```
linear_velocity here corresponds to linear.x in Twist
angular_velocity here corresponds to angular.z in Twist

__Publish a message using CMD line__ <br>
`rostopic pub -r 10 /turtle1/cmd_vel geometry_msgs/Twist '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'` where
- `pub` means publish.
- `-r 10` means repeat 10 times.
- `/turtle1/cmd_vel` is the topic.
- `geometry_msgs/Twist` is the message type.
- `'{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z :0.0}}'` is the data in json format.

__Misc__ <br>
`rostopic echo /turtle1/cmd_vel` outputs the content of the messages in `cmd_vel` topic when the message is published.

In turtlesim, the robot is only able to move using linear.x (forward, backward) and angular.z (rotate). This is sufficient for 2D motion.

### 3.4 Create Publisher/Subscriber Files
For new python files, allow the python file to be executed. <br>
- Right click on the file in file explorer
- Go to Permissions
- Check 'Allow executing file as program'
- Or enter `chmod +x <filename.py>` in the terminal
- Or enter `chmod 777 <filename.py>` in the terminal.

For new C++ files, modify `add_executable` in the CMakeLists.txt accordingly. <br>
The name of the nodes are defined in the CMakeLists `add_executable()`.

In ROS, nodes are uniquely named. If two nodes with the same node are launched, the previous one is kicked off. The `anonymous=True` flag means that rospy will choose a unique name for our 'listener' node so that multiple listeners can run simultaneously.

In C++ implementation, Message type is not defined in node instantiation, but in the callback function.

__Write Publisher of ROS Topics__ <br>
1. Determine a name for topic.
2. Determine the type of the messages that the topic will be publish.
3. Determine the frequency of topic publication (per second).
4. Create a publisher object with the above parameters.
5. Keep publishing the topic message at the selected frequency.

__Write Subscriber to ROS Topics__<br>
1. Identify the name for the topic to listen to.
2. Identify the type of the messages to be received.
3. Define a *callback function* that will be executed when a new message is received.
4. Start listening for the topic messages.
5. Spin to listen forever (in C++).

__CMakeLists.txt__<br>
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

__package.xml__ <br>
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

### 3.5 Custom Message
Create a `msg` folder in the ROS package folder. <br>
Create a `.msg` file and add the fields using a text editor. <br>
Refer to [ROS Wiki](wiki.ros.org/msg) for the built-in data types. 

Structure: *package_name/message_type* <br>
Type 1 -> field 1, field 2, field 3 <br>
*linear -> x, y, z* <br>
Type 2 -> field 1, field 2, field 3 <br>
*angular -> x, y, z* <br>
string -> data

e.g. *IoTSensor.msg*
```
int32 id
string name
float32 temperature
float32 humidity
```

__Update Dependencies__ <br>
CMakeLists.txt
- Add `message_generation` as a dependency under `find_package`.
- Add the `.msg` file under `add_message_files`.
- Add `message_runtime` under `catkin_package` beside `CATKIN_DEPENDS`.

package.xml
- `<build_depend>message_generation</build_depend>
- `<exec_depend>message_runtime</exec_depend>

In terminal, `cd catkin_ws` and run the command `catkin_make`.

## Section 4: ROS Services

### 4.1 General

ROS Server, ROS Client. <br>
Synchronous, bi-directional (request & response message), one-time.

Service is a one-time communication. A client sends a request, and waits for the server to return a response. The client will only wait and not do anything else until the response is received, unless a timeout is used.

Use case: to request the robot to perform a specific action.

### 4.2 Commands

`rosservice list` displays the list of services available in the active node. <br>
`rosservice info /spawn` gives information about the specified service servers during runtime.

`rossrv list` displays the list of services in the workspace.<br>
`rossrv show <service>` displays all the packages with the specified service. <br>
`rossrv info turtlesim/Spawn` displays the service definitions (request args, response args) by accessing the .srv files, where
- `turtlesim` is the package
- `Spawn` is the message type.

`rosservice call <service> <service_arguments>` <br>
e.g. `rosservice call /spawn 7 7 0 turtle2` calls the `/spawn` service, where
- x -> 7
- y -> 7
- theta -> 0
- name -> turtle2

This service responds with the spawned turtle's name.

### 4.3 Custom ROS Service: Add 2 Integers

__Steps__
1. Define the service message (service file).
2. Create ROS Server node.
3. Create ROS Client node.
4. Execute the service.
5. Consume the service by the client

__Step 1 Define Service Message__<br>
Create .srv file for the service definitions, containing the request and response arguments. <br>
For each argument, include the data type and name. e.g.
```
int64 a
int64 b
---
int64 sum
```
Update dependencies in package.xml <br>
```
<build_depend>message_generation</build_depend>

<exec_depend>message_runtime</exec_depend>
```
Update dependencies in CMakeLists.txt <br>
- add `message_generation` under `find_package`
- add `file_name.srv` under `add_service_files`
- `cakin_make` in workspace directory to create the service

3 header files (.h) will be created in the workspace `devel/include/ros_essentials` for the service.

__Step 2 & 3 Create ROS Service/Client Node__ <br>
Refer to source files.

The python client can be executed if the C++ server is running.

For new python files, allow the python file to be executed. <br>
- Right click on the file in file explorer
- Go to Permissions
- Check 'Allow executing file as program'
- Or enter `chmod +x <filename.py>` in the terminal
- Or enter `chmod 777 <filename.py>` in the terminal.

For new C++ files, modify `add_executable` in the CMakeLists.txt accordingly.

## Section 5: ROS Motion

### 5.1 Motion Types

Linear (x,y,z), Angular. In 2D motion, there is only z-angular for yaw.

Moving in Straight Line
- Linear x: constant
- zero for everything else

Go to Goal
- Linear x: f(distance from goal)
- Angular z: f(angle from goal)
- PID controller

Spiral
- Linear x: f(time)
- Angular z: constant

### 5.2 Implementation
Step 1: Understand topics (cmd_vel, pose) and messages to be used (Twist, Pose)

__Divide & Conquer Approach__
|Step|Description|
|--:|-----------|
|1|Develop a function to move in a *straight line* for a certain distance, forward and backward.|
|2|Develop a function to *rotate* in place for a certain angle, closewise and counter-clockwise.|
|3|Develop a function to go go a goal location.|
|4|Develop a function to move in spiral shape |
|5|Integrate all the above to develop the cleaning application. |

`loop_rate = rospy.Rate(10)` <br>
If the rate is low (e.g. 1 Hz), the robot's position is updated less often while it moves continuously, and it may overshoot its goal. <br>
Balance between speed and rate to get acceptable error, where lower speed requires lower rate.

__Logging__ <Br>
Python: `rospy.loginfo()` <br>
C++: `ROS_INFO()`

