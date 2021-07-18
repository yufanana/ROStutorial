#!/usr/bin/env python

import numpy as np
import cv2

def filter_color(rgb_image, lower_bound_color, upper_bound_color):
    #convert the image into the HSV color space
    hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)

    #define a mask using the lower and upper bounds of the yellow color 
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
    cv2.imshow("RGB Image Contours",rgb_image)

def get_contour_center(contour):
    M = cv2.moments(contour)
    cx=-1
    cy=-1
    if (M['m00']!=0):
        cx= int(M['m10']/M['m00'])
        cy= int(M['m01']/M['m00'])
    return cx, cy

def main():
    # create capture object from file
    video_capture = cv2.VideoCapture('video/tennis-ball-video.mp4')

    while(True):
        ret, frame = video_capture.read()
        
        if ret == False:
            print('ret is False')
            break
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print('Q pressed. Exiting...')
            break
        
        # ball detection algorithm
        yellowLower = (30, 150, 100)
        yellowUpper = (50, 255, 255)
        binary_frame_mask = filter_color(frame, yellowLower, yellowUpper)
        contours = getContours(binary_frame_mask)
        draw_ball_contour(binary_frame_mask, frame,contours)

    video_capture.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()