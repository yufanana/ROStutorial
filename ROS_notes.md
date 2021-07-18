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

To launch another .launch file,
```xml
<launch>
  <include file="$(find ros_essentials)/src/topic02_motion/launch/turtlesim_teleop.launch"/>

  <node name="cleaner_node" pkg="ros_essentials" type="turtlesim_cleaner.py" />
</launch>
```

To include parameteres,
```xml
<param name="x_goal" value="3.0"/>
<param name="y_goal" value="7.0"/>
```

Then in the python file,
```python
x_goal = rospy.get_param("x_goal")
y_goal = rospy.get_param("y_goal")
```

Add `output = "screen"` to the `<node>` to display output.

### 2.5 ROS Network Configuration
For ROS Indigo(?). No idea what's this for

User Workstation: 
- `gedit .bashrc`
- `ifconfig` to get IP address / inet addr
- Under #ROBOT MACHINE CONFIGURATION, `export ROS_HOSTNAME` and `export ROS_IP`, paste workstation's IP address
- For `export ROS_MASTER_URI`, paste the robot machine's IP address

Robot Machine: 
- `gedit .bashrc`
- `ifconfig` to get IP address / inet addr
- Under #ROBOT MACHINE CONFIGURATION, `export ROS_HOSTNAME` and `export ROS_IP`, paste robot machine's IP address for 
- ROS_MASTER_URI for robot machine will be on localhost

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

`rospy.init_node('noade_name,anonymous = True)` to create a new node with the specified name. 

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

Syntax: `add_executable(name_of_exe_file relative/path/to/source_file.cpp)`

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

## Section 5: Computer Vision with OpenCV

### 5.1 Applications

__Image Segmentation__ <br>
The process of partitioning a digital image into multiple segments. <br>
Used to locate objects and boundaries (e.g. lines, curves) in images. <br>

__Image Thresholding__<br>
Simplest method of image segmentation. <br>
Take a colour as a threshold <br>
- any colour above threshold -> white
- any colour below threshold -> black

This process creates binary images.

__Object Detection and Recognition__ <br>
Detecting instances of semantic objects of a certain class in digital images and videos.

__Drawing__ <br>
Drawing shapes like lines, polygons, text, circles.

__Edge Detection__ <br>
Find the boundaries of objects within images. <br>
Works by detecting discontinuities in brightness. <br>
Used for image segmentation and data extraction. <br>

__Video/Image Input/Output__ <br>
Read/write images and video streams.

__Installation__ <br>
I followed the instructions here to build OpenCV <br>
http://www.codebind.com/cpp-tutorial/install-opencv-ubuntu-cpp/

### 5.2 OpenCV (no ROS)

numpy is multidimensional array data structure is used to store pixel values of images.

__Executing Python files__ <br>
Include `#!/usr/bin/env python` at the top of the file. Then, enter `./file_name.py` in the terminal to run the specified file in the current directory.

OR

Enter `python3 file_name.py` in the terminal.

__Open/Save Image__ <br>
`color_img = cv2.imread("path/to/img.jpg", CV_LOAD_IMAGE_COLOR)` for colour image. <br>
`gray_img = cv2.imread("path/to/img.jpg", CV_LOAD_IMAGE_GRAYSCALE)` for grayscale image.

`cv2.namedWindow("window_name", cv2.WINDOW_NORMAL)` to create a window holder for image. The window can be referenced later to be moved, resized, closed, etc. <br>
`cv2.moveWindow("window_name",x_pos,y_pos)` to move the window to specified location.

`cv2.destroyAllWindows()` to destroy all windows. Usually called after the quit wait key.

`cv2.imshow("window_name",img)` to display image in the specified window. <br>
Should call `cv2.waitKey(1)` to allow high GUI some time to process the draw requests from `cv2.imshow()`.

`cv2.imwrite("path/to/file"+img_name+".jpg",img)` to save the image as specified file name at specified path.

`height, length, channels = img.shape` gives the dimensions of the numpy array. Channels are for colour images.<br>
`img[:,:,0]` to return all the values in the first channel.

`img.dtype` to obtain image datatype.

__Image Encoding__ <br>
- Grayscale 
- Red, Green, Blue (RGB)
- Hue, Saturation, Value (HSV)
  - Hue: 0-360, indicates type of colour
  - Saturation: 0-100%, amount of gray in colour
  - Value: 0-100%, brightness level, with 0 as black and 100 as most colour

<img src="./notes_images/HSV_cylinder.png" height=200>

OpenCV uses different ranges for HSV. <br>
- Hue: 0-180
- Saturation: 0-255
- Value: 0-255

<img src="./notes_images/HSV_color_space.jpg" height=150>


`blue,green,red = cv2.split(color_image)` to split image into the 3 channels. <br>
Then, you can go on to show each channel image.

`stiched_img = np.concatenate((array1,array2,array3),axis=1)` to concatenate the arrays along its length. Use `axis=0` to concatenate along its height.

`gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)` to convert color to grayscale. <br>
`hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)` to convert color to HSV.

__Video Stream Input__ <br>
`video_capture = cv2.VideoCapture(0)` to open a camera for video capturing. <br>
`video_capture.release()` Ensure that the capture object is released subsequently before exiting the script. 

`ret,frame = video_capture.read()` where frame is the image from the video, and ret is the return value. ret becomes false if no frames has been grabbed (camera disconnected, end of video file)



Use 'Q' button to break the while-loop and exit the script. <br>
```python
if cv2.waitKey(1) & 0xFF == ord('q'):
    break
```

__Drawing__ <br>
Points are represented as tuples: `(x1,y1)`, `(x2,y2)`<br>
Color is represented in BGR tuple: `(255,0,0)` <br>
Thickness is an integer: draws filled if negative, outline if positive

`cv2.rectangle(image,pt1,pt2,color,thickness)` to draw a rectangle.

`cv2.line(image,pt1,pt2,color,thickness)` to draw a line.

Axes is a tuple: (major_axis_length, minor_axis_length) <br>
`cv2.ellipse(image,center_pt,axes,angle,startAngle,endAngle,color,thickness)` to draw an ellipse.

`cv2.circle(image,center_pt,radius,color,thickness)` to draw a circle.

Origin is the bottom-left corer of the text string. <br>
`cv2.putText(image,text,orgin,font_type,font_size,color,thickness)` to put text on the image.

__CvBridge__ <br>
The image file produced by ROS (ROS Image Message) is not immediately compatible with the OpenCV format (OpenCV cv::Mat). Thus, CvBridge is needed to do the conversion (bidirectional).

`bridge = CvBridge()` to make a bridge object.

Inside the `image_ballback(ros_image)` function
```python
from cv_bridge import CvBridge, CvBridgeError

# convert ros img_msg to cv_image (e.g. in scubscriber)
try:
    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
except CvBridgeError as e:
    print(e)
  
# convert cv_image to ros img_msg (e.g. in publisher_)
try:
    ros_img = bridge.cv2_to_imgmsg(cv_image, encoding = "passthrough")
except CvBridgeError as e:
    print(e)
```
OpenCV operations can be applied to cv_image after this step.

`rosrun usb_cam usb_cam_node _pixel_format:= yuyv` to run the usb_cam


__Thresholding__ <br>
Simplest method of image segmentation. <br>
Take a colour as a threshold to compare against the pixel values. <br>

`cv2.threshold(gray_image,threshold_value,max_value,threshold_style)` to run simple thresholding.<br>
max_value is typically 255 (?)

Threshold styles
- THRESH_BINARY
- THRESH_BINARY_INV
- THRESH_TRUNC
- THRESH_TOZERO
- THRESH_TOZERO_INV

Simple thresholding may not be good in all lighting conditions (e.g. shadow in a section of image) 

Adaptive thresholding:
- calculates threshold for a small region of the image
- different thresholds calculated for different regions of the same image
- greater robustness against varying illumination

`cv2.adaptive_thresholding(gray_image,max_value,adaptive_method,block_size,constant)` to run adaptive thresholding. <br>
Block size: size of neighbourhood area <br>
Constant: constant that is subtracted from the mean/weight mean calculated

Adaptive styles
- ADAPTIVE_THRESH_MEAN_C
- ADAPTIVE_THRESH_GAUSSIAN_C

__Color Filtering__ <br>
The process displays only a specific color range in the image. <br>
This allows for the detection of objects with specific colors. <br>
HSV is used for filtering because it is more robust against external lighting conditions. <br>
Similar colors will be closer within the a range (angles) in HSV than in RGB.

Algorithm
- Read image as RGB
- Convert image to HSV
- Define upper and lower color ranges
- Create the mask based on color ranges

OpenCV has its own convention to define the ranges of hue, saturation and value.

__Tennis Ball Example__ <br>
1. Choose the yellow angle range
```python
yellowLower =(30, 150, 100)
yellowUpper = (50, 255, 255)
```
2. Saturation and value ranges are can be obtained via trial & error. Generally on the higher side.

__Contour Detection__ <br>
Contours: curves with the same color/intensity that join all the continuous points along a boundary

To find the boundaries of objects within image. <br>
This is done by detecting discontinuities in brightness. <br>
Useful for shape detection, image segmentation, object detection/recognition.

Algorithm
- Read image as RGB
- Convert image to grayscale
- Convert gray image to binary image
- Find contours using `cv2.findContours()` on the binary image
- Process the contours (e.g. area, centroid, perimeter, moment)

Contour Hierarchy <br>
Outer shape: parent, inner shape: child.

Contour Retrieval Modes (RETR: retrieve)
- `RETR_LIST`: returns all contours without parent-child relationships
- `RETR_EXTERNAL`: returns extreme outer contours only
- `RETR_CCOMP`: returns all contours, arranged in a 2-level hierarchy
- `RETR_TREE`: returns all contours in full family hierarchy

`contours, hierarchy = cv2.findContours(binary_img, contour_retr_mode, contour_approx_method)`

__Contour Processing__ <br>
Can create a blank black image to overlay processed contours and check the results.

Common operations
```python
area = cv2.contourArea(c)
perimeter= cv2.arcLength(c, True)
((x, y), radius) = cv2.minEnclosingCircle(c)  # returns the circle with min area that fully contains the contour

def get_contour_center(contour):
    M = cv2.moments(contour)
    cx=-1
    cy=-1
    if (M['m00']!=0):
        cx= int(M['m10']/M['m00'])
        cy= int(M['m01']/M['m00'])
    return cx, cy
```

__Tennis Ball Detection__ <br>

Steps
1. Read image as RGB
2. Apply color filtering to get a binary image mask
3. Generate contours using the binary image
4. Draw contours that are sufficiently large

__Tennis Ball Tracking__ <br>

Steps
1. Read image as RGB
2. Apply color filtering to get a binary image mask
3. Generate contours using the binary image
4. Draw contours that are sufficiently large

__OpenCV C++ Implementation__ <br>

CMakeList.txt
```
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(read_video_cpp src/topic03_perception/cpp/read_video.cpp)
target_link_libraries(read_video_cpp ${catkin_LIBRARIES})
target_link_libraries(read_video_cpp ${OpenCV_LIBRARIES})
```


__OpenCV & ROS C++ Implementation__ <br>
For C++ OpenCV Implementation with ROS, <br>
a `image_transport::ImageTransport it_;` is used to create a subscriber/advertiser instead of a node to handle images.

CMakeList.txt
```
find_package(
  roscpp
  std_msgs
  OpenCV
  cv_bridge
  image_transport
)

include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(image_pub_sub src/topic03_perception/cpp/image_pub_sub.cpp)
target_link_libraries(image_pub_sub ${catkin_LIBRARIES})
target_link_libraries(image_pub_sub ${OpenCV_LIBRARIES})
```



cd ros_essentials/src/topic03_perception/

