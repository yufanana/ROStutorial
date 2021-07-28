#!/usr/bin/env python

import rospy
import tf
import math
import geometry_msgs.msg

goal_location = (0,2)

def go_straight():

    return

def follow_wall():

    return

if __name__ == '__main__':
    #init node
    rospy.init_node('turtle_tf_listener')

    #create a new transform listerner
    transform_listener = tf.TransformListener()

    #turtle1 already created by turtlesim_node
    #create a second turtle by calling the service
    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    spawner(4, 2, 0, 'turtle2')

    turtle_follower_velocity = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            # tf of turtle2 with respect to turtle1
            (translation,rotation) = transform_listener.lookupTransform('/turtle2_frame', '/turtle1_frame', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        x_follower_in_turtle1_frame = translation[0]
        y_follower_in_turtle1_frame = translation[1]

        angular = 4 * math.atan2(y_follower_in_turtle1_frame, x_follower_in_turtle1_frame)
        linear = 0.5 * math.sqrt(x_follower_in_turtle1_frame ** 2 + y_follower_in_turtle1_frame ** 2)
        
        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        turtle_follower_velocity.publish(cmd)

        rate.sleep()

