#!/usr/bin/env python

# roslaunch turtlebot3_gazebo turtlebot3_stage_4.launch

'''
This Bug0 alrogithm turns left upon encountering an obstacle.
'''

import rospy
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from tf import transformations

srv_client_go_to_point_ = None
srv_client_wall_follower_ = None
yaw_ = 0
yaw_error_allowed_ = 5 * (math.pi / 180) # 5 degrees

# Precision of facing/reaching goal
yaw_precision_ = math.pi / 90 # +/- 2 degree allowed
dist_precision_ = 0.3

position_ = Point()
#initial_position_ = Point()
#initial_position_.x = rospy.get_param('initial_x')
#initial_position_.y = rospy.get_param('initial_y')
#initial_position_.z = 0

#regions_ = None
state_desc_ = ['Go to point', 'wall following']
state_ = 4  # initialise state as go to goal
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
    3: 'fix yaw',
    4: 'go to goal',
    5: 'reached goal'
}

def odom_callback(msg):
    '''
    Updates the robot's current position that is stored in the global variable
    '''
    global position_, yaw_
    
    # position
    position_ = msg.pose.pose.position
    
    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]
    return

def laser_callback(msg):
    '''
    Reads the scan, determines where are the obstacles
    '''
    #global regions_
    
    # define directions
    regions_ = {
        'front': min(min(msg.ranges[0:22]),min(msg.ranges[338:360]), 10),
        'fleft': min(min(msg.ranges[23:67]), 10),
        'left':  min(min(msg.ranges[68:102]), 10),
        'right': min(min(msg.ranges[247:291]), 10),
        'fright': min(min(msg.ranges[292:337]), 10),
    }

    # print("front: ",msg.ranges[0])
    # print("left: ",msg.ranges[90])
    # print("back: ",msg.ranges[180])
    # print("right: ",msg.ranges[270])
    # print("------")

    d = 0.5  # threshold to consider obstacle as far
    
    # determine direction of obstacle(s)
    if regions_['front'] > d and regions_['fleft'] > d and regions_['fright'] > d:
        state_description = 'case 1 - nowhere'
        change_state(0)     # find wall
    elif regions_['front'] < d and regions_['fleft'] > d and regions_['fright'] > d:
        state_description = 'case 2 - front only'
        change_state(1)     # turn left
    elif regions_['front'] > d and regions_['fleft'] > d and regions_['fright'] < d:
        state_description = 'case 3 - fright only'
        change_state(2)     # follow wall
    elif regions_['front'] > d and regions_['fleft'] < d and regions_['fright'] > d:
        state_description = 'case 4 - fleft only'
        change_state(0)     # find wall
    elif regions_['front'] < d and regions_['fleft'] > d and regions_['fright'] < d:
        state_description = 'case 5 - front and fright'
        change_state(1)     # turn left
    elif regions_['front'] < d and regions_['fleft'] < d and regions_['fright'] > d:
        state_description = 'case 6 - front and fleft'
        change_state(1)     # turn left
    elif regions_['front'] < d and regions_['fleft'] < d and regions_['fright'] < d:
        state_description = 'case 7 - front and fleft and fright'
        change_state(1)     # turn left
    elif regions_['front'] > d and regions_['fleft'] < d and regions_['fright'] < d:
        state_description = 'case 8 - fleft and fright'
        change_state(0)     # find wall
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions_)

def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print('Bug Zero - [%s] - %s' % (state, state_dict_[state]))
        state_ = state  # change global state to current state

def find_wall():
    msg = Twist()
    msg.linear.x = 0.1
    msg.angular.z = -0.1
    return msg

def turn_left():
    '''
    Turn left to start going along the wall
    '''
    cmd = Twist()
    cmd.angular.z = 0.3
    cmd.linear.x = 0
    return cmd

def follow_wall():
    '''
    Go straight along the wall
    '''
    cmd = Twist()
    cmd.angular.z = 0
    cmd.linear.x = 0.1
    return cmd

def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def fix_yaw(goal):
    '''
    Rotate the robot to face the goal
    '''
    global yaw_, yaw_precision_, state_
    desired_yaw = math.atan2(goal.y - position_.y, goal.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    
    rospy.loginfo(err_yaw)
    
    cmd = Twist()
    if math.fabs(err_yaw) > yaw_precision_:
        cmd.angular.z = 0.1 if err_yaw > 0 else -0.1
    
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_:
        print('Yaw error: [%s]' % err_yaw)
        cmd.angular.z = 0
        cmd.linear.x = 0
        change_state(4)

    return cmd

def go_to_goal(goal):
    '''
    
    '''
    global yaw_, yaw_precision_, state_
    desired_yaw = math.atan2(goal.y - position_.y, goal.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(goal.y - position_.y, 2) + pow(goal.x - position_.x, 2))
    
    if err_pos > dist_precision_:
        cmd = Twist()
        cmd.linear.x = 0.1
        cmd.angular.z = 0.1 if err_yaw > 0 else -0.1
    else:
        print('Position error: [%s]' % err_pos)
        change_state(5)
    
    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        print('Yaw error: [%s]' % err_yaw)
        change_state(0)
    return cmd

def main():
    global bug_velocity, position_
    goal = Point()
    goal.x = 0
    goal.y = 2
    desired_yaw = math.atan2(goal.y - position_.y, goal.x - position_.x)

    #init node
    rospy.init_node('bug0')
    
    sub_odom = rospy.Subscriber('/odom', Odometry, odom_callback)
    sub_laser = rospy.Subscriber('/scan', LaserScan, laser_callback)
    pub_velocity = rospy.Publisher('/cmd_vel',Twist, queue_size=1)
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        
        # publish cmd messages according to the current state
        if state_ == 0:
            cmd = find_wall()
        elif state_ == 1:
            cmd = turn_left()
        elif state_ == 2:
            cmd = follow_wall()
        elif state_ == 3:
            cmd = fix_yaw(goal)
        elif state_ == 4:
            cmd = go_to_goal(goal)
        elif state_ == 5:
            pass
        else:
            rospy.logerr('Unknown state!')

        pub_velocity.publish(cmd)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("\n Quitting...")
        pass