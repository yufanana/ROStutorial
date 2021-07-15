#!/usr/bin/env python

import numpy as np
import cv2

image_name = "flower"

# read an image from file
img = cv2.imread("images/"+image_name+".jpg")

# create window holder for the image
cv2.namedWindow("Image Viewer",cv2.WINDOW_NORMAL)

# display the image
cv2.imshow("Image Viewer",img)

# wait for key press before proceeding
print ('press a key inside the image to make a copy')
cv2.waitKey(0)

# write image to folder images/copy/
cv2.imwrite("images/copy/"+image_name+"-copy.jpg",img)
