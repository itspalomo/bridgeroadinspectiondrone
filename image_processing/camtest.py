import cv2
import numpy as np
import time
import os

#every 5s until keyboard interrup, pi will take a picture
def data_collection(cam_path: int, count: int, fname: str):
    cap = cv2.VideoCapture(cam_path, cv2.CAP_V4L2)
    
    i = 79

    while(cap.isOpened()):
        ret, frame = cap.read()
        
        # This condition prevents from infinite looping
        # incase video ends.
        if (ret == False):
            print('ret is False')
            break
        
        # Save Frame by Frame into disk using imwrite method
        cv2.imwrite(fname+str(i)+'.jpg', frame)
        print('Photo Collected!')
        time.sleep(5)
        i += 1
    cv2.waitKey(1)
    cap.release()
    cv2.waitKey(1)

def ir_data_collection(fname: str):    
    i = 79
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

    while(True):
        os.system('seek_snapshot -t seekpro -o ' +fname+str(i)+'ir.png -w 3 -c 4 -r 180')
        print('Photo Collected!')
        ret, frame = cap.read()
        if ret:
            cv2.imwrite(fname+str(i)+'.jpg', frame)
            print('Visible Light Photo Collected!')
        time.sleep(3)
        i += 1
    

def cam_ind_test(cam_path: int, count: int, fname: str):
    cap = cv2.VideoCapture(cam_path, cv2.CAP_V4L2)
    
    i = 0

    while(cap.isOpened()):
        ret, frame = cap.read()
        
        # This condition prevents from infinite looping
        # incase video ends.
        if (ret == False) or (i > count):
            print('ret is False')
            break
        
        # Save Frame by Frame into disk using imwrite method
        cv2.imwrite(fname+str(i)+'vl.png', frame)
        i += 1
    cv2.waitKey(1)
    cap.release()
    cv2.waitKey(1)

#data_collection(0,5, 'vl')
ir_data_collection('/home/ubuntu/ros2_ws/bridgeroadinspectiondrone/image_processing/vl_pics_10_25/photo_')