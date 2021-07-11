#!/usr/bin/env python

from ros_service_assignment.srv import RectangleArea
from ros_service_assignment.srv import RectangleAreaRequest
from ros_service_assignment.srv import RectangleAreaResponse

import rospy

def handle_get_area(request):
    print("Returning area of [width: {}, height: {}]".format(request.width, request.height))
    return RectangleAreaResponse(request.width * request.height)

def rect_area_server():
    rospy.init_node('rect_area_server')
    server = rospy.Service('get_rect_area',RectangleArea,handle_get_area)
    print("Ready to get rectangle area.")
    rospy.spin()

if __name__ == "__main__":
    rect_area_server()