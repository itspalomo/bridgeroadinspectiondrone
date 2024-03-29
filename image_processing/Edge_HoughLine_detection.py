import cv2
import numpy as np
import argparse
import os

os.chdir("Road_Videos")       #change into whatever directory your test videos are stored in
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--media", required=True,
	help="path to input video or image")
args = vars(ap.parse_args())

#start streaming video
vs = cv2.VideoCapture(args["media"])
#vs = cv2.VideoCapture(0).   #uncomment to analyze live video feed

while True:
    ret,frame = vs.read()
    cv2.imshow("Original", frame)
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    edge = cv2.Canny(img,300,350)
    lines =cv2.HoughLinesP(edge, rho = 1, theta = 1*np.pi/180, threshold = 100, minLineLength = 170, maxLineGap = 10)

    for i in lines:
        x1,x2,y1,y2 = i[0]
        cv2.line(frame,(x1,x2), (y1,y2), (0,255,0),3)
    cv2.imshow("media", frame)

    key = cv2.waitKey(1)
    if key == ord("q"):
        break

vs.release()
cv2.destroyAllWindows()
