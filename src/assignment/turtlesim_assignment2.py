#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

import math
import time
from std_srvs.srv import Empty

x = 0
y = 0
z = 0
yaw = 0

def poseCallback(pose_message):
    global x
    global y, z, yaw
    x = pose_message.x
    y = pose_message.y
    yaw = pose_message.theta

    print("Pose callback" + rospy.get_caller_id())
    print("x = {}".format(pose_message.x))
    print("y = {}".format(pose_message.y))
    print("yaw = {}".format(pose_message.theta))

def move(speed, distance):
    # declare a Twist message to send velocity commands
    velocity_message = Twist()

    # instantiate helper variables
    x0 = x
    y0 = y
    distance_moved = 0.0

    velocity_message.linear.x = speed
    loop_rate = rospy.Rate(10)

    # create publisher
    velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)

    while True:
        rospy.loginfo("Turtlesim moves forward")
        # publish the message
        velocity_publisher.publish(velocity_message)

        loop_rate.sleep()

        distance_moved = distance_moved + abs(0.5 * math.sqrt(((x-x0) ** 2) + ((y-y0) ** 2)))
        print(distance_moved)

        if not(distance_moved < distance):
            rospy.loginfo("Reached")
            break

    # stop the robot after reaching specified distance
    velocity_message.linear.x = 0
    velocity_message.publish(velocity_message)

if __name__ == '__main__':
    try:
        
        rospy.init_node('turtlesim_motion_pose', anonymous=True)

        #declare velocity publisher, pose subscriber
        velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        pose_subscriber = rospy.Subscriber("/turtle1/pose", Pose, poseCallback) 

        time.sleep(2)
        print("move: ")
        move(1.0, 5.0)
        
        time.sleep(2)
        print("start reset: ")
        rospy.wait_for_service('reset')
        reset_turtle = rospy.ServiceProxy('reset', Empty)
        reset_turtle()
        print("end reset: ")
        rospy.spin()
       
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")