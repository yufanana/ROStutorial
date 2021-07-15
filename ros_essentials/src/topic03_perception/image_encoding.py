#!/usr/bin/env python 

import numpy as np
import cv2


image_name = "tree"

print ('read an image from file')
color_image = cv2.imread("images/"+image_name+".jpg",cv2.IMREAD_COLOR)

print ('display image in native color')
cv2.imshow("Original Image",color_image)
cv2.moveWindow("Original Image",0,0)
print(color_image.shape)

height,width,channels = color_image.shape

# split image into three channels.
blue,green,red = cv2.split(color_image)

# concatenate to combine 3 arrays into 1 image
channel_imgs = np.concatenate((blue,green,red),axis=1)
cv2.imshow("Blue, Green, Red Channel Image",channel_imgs)

#cv2.imshow("Blue Channel",blue)
#cv2.moveWindow("Blue Channel",250,0)

#cv2.imshow("Red Channel",red)
#cv2.moveWindow("Red Channel",500,0)

#cv2.imshow("Greeen Channel",green)
#cv2.moveWindow("Green Channel",750,0)

# Hue: indicates the type of color that we see in a 360 degree format.
# Saturation: an indication of how saturated an individual color is 
# Value: indicates how luminous the channel is. 

# convert image to HSV
hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
h,s,v = cv2.split(hsv)
hsv_image = np.concatenate((h,s,v),axis=1)
cv2.imshow("Hue, Saturation, Value Image",hsv_image)
cv2.imshow("HSV Image",hsv)
cv2.moveWindow("HSV Image",width*4,0)


# convert image to grayscale
gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
cv2.imshow("Gray Image ",gray_image)
cv2.moveWindow("Gray Image",width*5,0)

print (gray_image)

# close all windows conveniently
cv2.waitKey(0)
cv2.destroyAllWindows()
