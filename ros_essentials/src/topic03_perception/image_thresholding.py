#!/usr/bin/env python 

import numpy as np
import cv2
from numpy.core.numeric import count_nonzero

y = 0

def read_image(image_name, as_gray):
    global y
    # as_gray is Boolean to decide reading style
    if as_gray: 
        image = cv2.imread(image_name,cv2.IMREAD_GRAYSCALE)
    else:
        image = cv2.imread(image_name,cv2.IMREAD_COLOR)
    width = image.shape[1]
    window_name = "Image {}".format(y)
    cv2.namedWindow(window_name)
    cv2.moveWindow(window_name,width,y)
    cv2.imshow(window_name,image)
    return image

def basic_thresholding(gray_image, threshold_value):
    global y
    ret, thresh_basic = cv2.threshold(gray_image,
                                    threshold_value,
                                    255,
                                    cv2.THRESH_BINARY_INV)
    width = gray_image.shape[1]
    window_name = "Basic Binary Image {}".format(y)
    cv2.namedWindow(window_name)
    cv2.moveWindow(window_name, width*2, y)
    cv2.imshow(window_name,thresh_basic)

def adaptive_thresholding(gray_image, threshold_value):
    global y
    adaptive_threshold_image = cv2.adaptiveThreshold(gray_image, 
                                        255, 
                                        cv2.ADAPTIVE_THRESH_MEAN_C, 
                                        cv2.THRESH_BINARY_INV, 
                                        threshold_value, 
                                        2)
    width = gray_image.shape[1]
    window_name = "Adaptive Threshold Image {}".format(y)
    cv2.namedWindow(window_name)
    cv2.moveWindow(window_name, width*3, y)
    cv2.imshow(window_name,adaptive_threshold_image)


def main(image_name):
    as_gray = True
    threshold_value=115
    gray_image = read_image(image_name,as_gray)
    basic_thresholding(gray_image, threshold_value)
    adaptive_thresholding(gray_image, threshold_value)
    global y
    y += gray_image.shape[0]

if __name__ == '__main__':
    image_name = "images/shapes.png"
    main(image_name)
    image_name = "images/tomato.jpg"
    main(image_name)
    cv2.waitKey(0)
    print("Exiting...")
    cv2.destroyAllWindows()