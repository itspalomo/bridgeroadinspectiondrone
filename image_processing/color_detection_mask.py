import cv2
import numpy as np
import argparse
import os

os.chdir("Road_Videos")

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--media", required=True,
	help="path to input video or image")
args = vars(ap.parse_args())

#cap = cv2.VideoCapture(0)
cap = cv2.VideoCapture(args["media"])

while True:
    _, frame = cap.read()
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Red color
    low_red = np.array([161, 155, 84])
    high_red = np.array([179, 255, 255])
    red_mask = cv2.inRange(hsv_frame, low_red, high_red)
    red = cv2.bitwise_and(frame, frame, mask=red_mask)
        # Blue color
    low_blue = np.array([94, 80, 2])
    high_blue = np.array([126, 255, 255])
    blue_mask = cv2.inRange(hsv_frame, low_blue, high_blue)
    blue = cv2.bitwise_and(frame, frame, mask=blue_mask)
        # Green color
    low_green = np.array([40, 60, 72])
    high_green = np.array([80, 255, 255])
    green_mask = cv2.inRange(hsv_frame, low_green, high_green)
    green = cv2.bitwise_and(frame, frame, mask=green_mask)
        # Yellow color
    low_yellow = np.array([20, 80, 70])
    high_yellow = np.array([33, 255, 255])
    yellow_mask = cv2.inRange(hsv_frame, low_yellow, high_yellow)
    yellow = cv2.bitwise_and(frame, frame, mask=yellow_mask)
        # White color
    low_white = np.array([0, 0, 0])
    high_white = np.array([179, 15, 255])
    white_mask = cv2.inRange(hsv_frame, low_white, high_white)
    white = cv2.bitwise_and(frame, frame, mask=white_mask)
        # White and Yellow color
    low_WY = np.array([20, 0, 70])
    high_WY = np.array([33, 255, 255])
    WY_mask = cv2.inRange(hsv_frame, low_WY, high_WY)
    WhiteYellow = cv2.bitwise_and(frame, frame, mask=WY_mask)
    #frame = cv2.Canny(frame, 150, 200)
        # Black color
    low_black = np.array([0, 0, 0])
    high_black = np.array([179, 255, 30])
    black_mask = cv2.inRange(hsv_frame, low_black, high_black)
    black = cv2.bitwise_and(frame, frame, mask=black_mask)    
        # Every color except white
    low = np.array([0, 42, 0])
    high = np.array([179, 255, 255])
    mask = cv2.inRange(hsv_frame, low, high)
    result = cv2.bitwise_and(frame, frame, mask=mask)
    #draw bounding rectangle around yellow colors
###################################
    low_yellowBox = np.array([20, 80, 70])
    high_yellowBox = np.array([33, 255, 255])
    yellowBox_mask = cv2.inRange(hsv_frame, low_yellowBox, high_yellowBox)
    yellowBox = cv2.bitwise_and(frame, frame, mask=yellowBox_mask)
    contours, hierarchy = cv2.findContours(yellowBox_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
      
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(100000 > area >= 3000):     #area > 300
            x, y, w, h = cv2.boundingRect(contour)
            yellowBox = cv2.rectangle(yellowBox, (x, y), 
                                       (x + w, y + h),
                                       (255, 255, 255), 2)
              
            cv2.putText(yellowBox, "Yellow", (x, y-20),
                        cv2.FONT_HERSHEY_SIMPLEX, 
                        2, (255, 255, 255), 3)
###################################


    cv2.imshow("Original", frame)
    cv2.imshow("Red", red)
    cv2.imshow("Blue", blue)
    cv2.imshow("Green", green)
    cv2.imshow("Yellow", yellow)
    cv2.imshow("Yellow with bounding boxes", yellowBox)
    cv2.imshow("White", white)
    cv2.imshow("White and Yellow", WhiteYellow)
    cv2.imshow("Black", black)
    cv2.imshow("Result", result)

    key = cv2.waitKey(1)
    if key == 27:
        break