import cv2
import numpy as np
import time

#every 5s until keyboard interrup, pi will take a picture
def cam_ind_test(cam_path: int, count: int, fname: str):
    cap = cv2.VideoCapture(cam_path, cv2.CAP_FFMPEG)
    
    i = 0

    while(cap.isOpened()):
        ret, frame = cap.read()
        
        # This condition prevents from infinite looping
        # incase video ends.
        if (ret == False) or (i > count):
            print('ret is False')
            break
        
        # Save Frame by Frame into disk using imwrite method
        cv2.imwrite(fname+str(i)+'.jpg', frame)
        i += 1
    cv2.waitKey(1)
    cap.release()
    cv2.waitKey(1)

cam_ind_test(0,5, 'vl')