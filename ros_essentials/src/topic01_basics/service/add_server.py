#!/usr/bin/env python

from ros_essentials.srv import AddTwoInts
from ros_essentials.srv import AddTwoIntsRequest
from ros_essentials.srv import AddTwoIntsResponse

import rospy

def handle_add_two_ints(req):
    # req is the request message. It has a & b arguments.
    print ("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
    # return is the response message
    return AddTwoIntsResponse(req.a + req.b)

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    # Creates a server called s
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    # add_two_ints: service name
    # AddTwoInts: service type
    # handle_add_two_ints: callback function
    print ("Ready to add two ints.")
    rospy.spin()
    
if __name__ == "__main__":
    add_two_ints_server()
