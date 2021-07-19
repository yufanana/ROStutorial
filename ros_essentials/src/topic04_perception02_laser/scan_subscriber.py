#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import math


def field_of_view(scan_data):
    return (scan_data.angle_max-scan_data.angle_min)*180.0/3.14

#find the max range and its index
def min_range_index(ranges):
    return (min(ranges), ranges.index(min(ranges)) )

#find the max range 
def max_range_index(ranges):
    return (max(ranges), ranges.index(max(ranges)) )

#find the average range
def average_range(ranges):
    return ( sum(ranges) / float(len(ranges)) )

def average_between_indices(ranges, i, j):
    slice_of_array = ranges[i: j+1]
    return ( sum(slice_of_array) / float(len(slice_of_array)) )

def scan_callback(scan_data):
    # discard all the nan values
    ranges = [x for x in scan_data.ranges if not math.isnan(x)]

    min_value, min_index = min_range_index(ranges)
    print("min range value: {}".format(min_value))
    print("min range index: {}".format(min_index))

    max_value, max_index = max_range_index(ranges)
    print("max range value: {}".format(max_value))
    print("max range index: {}".format(max_index))

    average_value = average_range(ranges)
    print("avg range value: {}".format(average_value))

    average2 = average_between_indices(ranges, 2, 7)
    print("avg between 2 indices: {}".format(average2))

    fov = field_of_view(scan_data)
    print("field of view: {}".format(fov))

if __name__ == '__main__':
    rospy.init_node('scan_node', anonymous=True)
    rospy.Subscriber("scan", LaserScan, scan_callback)
    rospy.spin()