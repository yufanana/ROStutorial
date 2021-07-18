#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys

bridge = CvBridge()

def filter_color(rgb_image, lower_bound_color, upper_bound_color):
    #convert the image into the HSV color space
    hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)

    #define a mask using the lower and upper bounds
    mask = cv2.inRange(hsv_image, lower_bound_color, upper_bound_color)

    return mask

def getContours(mask):      
    contours, hierarchy = cv2.findContours(mask.copy(), 
                                            cv2.RETR_EXTERNAL,
	                                        cv2.CHAIN_APPROX_SIMPLE)
    return contours


def draw_ball_contour(binary_image, rgb_image, contours):
    
    for c in contours:
        area = cv2.contourArea(c)

        # only draw contours that are sufficiently large
        if (area>3000):
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            cv2.drawContours(rgb_image, [c], -1, (150,250,150), 2)
            
            cx, cy = get_contour_center(c)
            cv2.circle(rgb_image, (cx,cy),(int)(radius),(0,0,255),2)
            print ("Area: {}".format(area))

    print ("number of contours: {}".format(len(contours)))
    cv2.namedWindow("RGB Image Contours",cv2.WINDOW_NORMAL)
    cv2.imshow("RGB Image Contours",rgb_image)

def get_contour_center(contour):
    M = cv2.moments(contour)
    cx=-1
    cy=-1
    if (M['m00']!=0):
        cx= int(M['m10']/M['m00'])
        cy= int(M['m01']/M['m00'])
    return cx, cy

def image_callback(ros_image):
    print('Received an image')
    global bridge

    #convert ros_image into cv image
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
        #cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
        #im_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
    except CvBridgeError as e:
        print(e)
  
    # detection algorithm
    yellowLower = (30, 150, 100)
    yellowUpper = (50, 255, 255)
    binary_frame_mask = filter_color(cv_image, yellowLower, yellowUpper)
    contours = getContours(binary_frame_mask)
    draw_ball_contour(binary_frame_mask, cv_image,contours)
    # call waitKey(1) to allow time for the window to draw
    cv2.waitKey(1)

  
def main(args):
  rospy.init_node('tennis_ball_listener', anonymous=True)
  rospy.Subscriber("tennis_ball_image",Image, image_callback)
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)