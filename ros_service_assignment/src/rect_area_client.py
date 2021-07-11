#!/usr/bin/env python

import sys
import rospy

from ros_service_assignment.srv import RectangleArea
from ros_service_assignment.srv import RectangleAreaRequest
from ros_service_assignment.srv import RectangleAreaResponse

def get_rect_area_client(width,height):
    rospy.wait_for_service('get_rect_area')
    try:
        client = rospy.ServiceProxy('get_rect_area',RectangleArea)
        response = client(width, height)
        return response.area
    except:
        print("Service call failed")

def usage():
    return "%s [width height]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        width = float(sys.argv[1])
        height = float(sys.argv[2])
    else:
        print("%s [width height]"%sys.argv[0])
        sys.exit(1)
    print("Requesting area of width: {}, height: {}".format(width,height))
    area = get_rect_area_client(width,height)
    print("For rectangle width: {}, height: {}, area: {}".format(width,height,area))