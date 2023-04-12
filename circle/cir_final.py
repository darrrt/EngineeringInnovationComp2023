# ！！！！！！！！！！！！！
import cv2
import numpy as np
from matplotlib import pyplot as plt
import math
cap = cv2.VideoCapture(2)
while(1):
    _, img = cap.read()
    # img=cv2.imread("11.jpg",3)
    # img=cv2.resize(img,(224*6,224*6),interpolation=cv2.INTER_CUBIC)
    img=cv2.resize(img,(640,480),interpolation=cv2.INTER_CUBIC)

    # img = cv2.medianBlur(img,1)
    # img=cv2.blur(img,(1,1))

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # lower_red = np.array([156,43,46])
    # upper_red = np.array([180,255,255])
    lower_red = np.array([0,43,46])
    upper_red = np.array([10,255,255])
    lower_green = np.array([50,43,46])
    upper_green = np.array([90,255,255])
    lower_blue = np.array([110,43,46])
    upper_blue = np.array([155,255,255])
    mask_red = cv2.inRange(hsv, lower_red, upper_red)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    mask=mask_green
    res = cv2.bitwise_and(img,img, mask= mask)
    img=mask
    cv2.imshow("0",img)

    num=10
    kernel = np.ones((2*num+1, 2*num+1), np.uint8)
    kernel_re = []
    rows, cols = kernel.shape
    for i in range(rows):
        result = [0 if math.sqrt((i-num)**2+(j-num)**2) > num else 1 for j in range(cols)]
        kernel_re.append(result)
    kernel_re = np.array(kernel_re, np.uint8)
    # kernel_re = np.ones((4, 4), np.uint8)
    #green 4 red 3 blue1
    for i in range(2):
        img = cv2.dilate(img, kernel_re, iterations = 2)
        # img = cv2.erode(img, kernel_re, iterations=1)
        # img = cv2.dilate(img, kernel_re, iterations = 2)
    for i in range(6):
        img = cv2.erode(img, kernel_re, iterations=1)
    cv2.imshow('1',img)

    # detected_circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, 1,10,param1=100,param2=30,minRadius=300,maxRadius=500)
    detected_circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, 1,1,param1=100,param2=12,minRadius=120,maxRadius=200)

    
    # Draw circles that are detected.
    if detected_circles is not None:
    
        # Convert the circle parameters a, b and r to integers.
        detected_circles = np.uint16(np.around(detected_circles))
        for pt in detected_circles[0, :]:
            a, b, r = pt[0], pt[1], pt[2]
            # Draw the circumference of the circle.
            cv2.circle(img, (a, b), r, (0,0, 0), 2)
    
            # Draw a small circle (of radius 1) to show the center.
            cv2.circle(img, (a, b), 1, (0, 0, 0), 3)
            cv2.imshow("Detected Circle", img)
    # cv2.imshow("0",img)
    if cv2.waitKey(1) & 0xff == ord('q'):
        break