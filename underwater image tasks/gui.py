#!/usr/bin/python3

import cv2
import numpy as np
import matplotlib.pyplot as plt
from vector_test import *
from shapes import *
from mapping import *

def cams (down,front):

    down_h,down_w,_=down.shape
    front_h,front_w,_=front.shape

    if (down_h > front_h) :
        height = down_h
        tolerance_d = 0
        tolerance_f = (down_h - front_h)//2
    else :
        height = front_h
        tolerance_d = (front_h - down_h)//2
        tolerance_f = 0

    width = front_w + down_w

    screen = np.zeros([height,width,3],np.uint8)

    screen[tolerance_d:down_h+tolerance_d,:down_w] = down
    screen[tolerance_f:front_h+tolerance_f,down_w:] = front

    return (screen)

def trackbars():

    def nothing(x) :
        pass

    cv2.namedWindow("track")
    cv2.createTrackbar("Vector","track",0,1,nothing)
    cv2.createTrackbar("Map","track",0,1,nothing)
    cv2.createTrackbar("Ship","track",0,1,nothing)

trackbars()

vector_length=0
vector_angle=0
vector_x=0
vector_y=0

dis_x = 20
dis_y = 20

while True :

    #data = open("data_input.txt",'r')

    #ret_down, down_frame = dowm_cam.read()
    #ret_front, front_frame = front_cam.read()

    down_frame = cv2.imread("test/test-5.jpg")
    front_frame = cv2.imread("test/real-3.jpg")    
    
    templets = shape_setup()

    vector_flag = cv2.getTrackbarPos("Vector","track")
    map_flag = cv2.getTrackbarPos("Map","track")
    ship_flag = cv2.getTrackbarPos("Ship","track")

    if (vector_flag == 1) :
        down,vector_length,vector_angle,vector_x,vector_y = vector(down_frame)
    else :
        down = down_frame

    if (ship_flag == 1) :
        front = shape(front_frame,templets)
    else :
        front = front_frame

    screen=cams(down,front)

    if (map_flag == 1) :
        draw_map(dis_x,dis_y,vector_length,vector_angle,vector_x,vector_y)

    text = np.ones([200,500,1],np.uint8) * 255

    text = cv2.putText(text,"city area= "+str(dis_x)+" * "+str(dis_y), (25,30), cv2.FONT_HERSHEY_SIMPLEX , 0.45 , 0, 1, cv2.LINE_AA)
    text = cv2.putText(text,"vector length= "+str(vector_length), (25,60), cv2.FONT_HERSHEY_SIMPLEX , 0.6 , 0, 1, cv2.LINE_AA)
    text = cv2.putText(text,"vector angle= "+str(vector_angle), (25,90), cv2.FONT_HERSHEY_SIMPLEX , 0.6 , 0, 1, cv2.LINE_AA)
    text = cv2.putText(text,"steps on x= "+str(vector_x), (25,120), cv2.FONT_HERSHEY_SIMPLEX , 0.6 , 0, 1, cv2.LINE_AA)
    text = cv2.putText(text,"steps on y= "+str(vector_y), (25,150), cv2.FONT_HERSHEY_SIMPLEX , 0.6 , 0, 1, cv2.LINE_AA)
    
    cv2.imshow("track",text)

    cv2.imshow("ROV_Vikings",screen)

    if cv2.waitKey(1) & 0xff == 27 :
        break

cv2.destroyAllWindows()