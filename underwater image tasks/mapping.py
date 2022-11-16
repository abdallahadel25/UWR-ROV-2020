#!/usr/bin/python3

import cv2
import numpy as np

def draw_map(dis_x,dis_y,vector_length,vector_angle,vector_x,vector_y):

    height = 500
    width = 500

    if dis_x > 20 :
        draw_x = 20
    else:
        draw_x = dis_x

    if dis_y > 20 :
        draw_y = 20
    else:
        draw_y = dis_y

    grid = np.ones((height,width,3),np.uint8) * 255

    width_step  = round(width/20)
    height_step = round(height/20)

    for step in range(width_step,width,width_step) :
        grid = cv2.line(grid,(step,0),(step,height),(0,0,0),1)

    for step in range(height_step,height,height_step) :
        grid = cv2.line(grid,(0,step),(width,step),(0,0,0),1)

    map = cv2.rectangle(grid,(0,height-(draw_y*height_step)),(draw_x*width_step,height),(255,0,0),5)
    map = cv2.ellipse(map,(0,height),(round(width_step*2.5),round(width_step*2.5)),270+(90-40),0,40,(0,0,0),2)
    map = cv2.arrowedLine(map,(0,height),(round(vector_x*width_step),height-round(vector_y*height_step)),(0,0,255),2,tipLength = 0.04)
    text = np.ones([500,300,3],np.uint8) * 255
    text = cv2.putText(text,"city area= "+str(dis_x)+" * "+str(dis_y), (20,30), cv2.FONT_HERSHEY_SIMPLEX , 0.45 , (255,0,0), 1, cv2.LINE_AA)
    text = cv2.putText(text,"vector length= "+str(vector_length), (20,60), cv2.FONT_HERSHEY_SIMPLEX , 0.6 , (255,0,0), 1, cv2.LINE_AA)
    text = cv2.putText(text,"vector angle= "+str(vector_angle), (20,90), cv2.FONT_HERSHEY_SIMPLEX , 0.6 , (255,0,0), 1, cv2.LINE_AA)
    text = cv2.putText(text,"steps on x= "+str(vector_x), (20,120), cv2.FONT_HERSHEY_SIMPLEX , 0.6 , (255,0,0), 1, cv2.LINE_AA)
    text = cv2.putText(text,"steps on y= "+str(vector_y), (20,150), cv2.FONT_HERSHEY_SIMPLEX , 0.6 , (255,0,0), 1, cv2.LINE_AA)

    final = np.ones([500,800,3],np.uint8) * 255

    final[:,width:] = text 
    final[:,:width] = map

    cv2.imshow("map",final)
    #return(final)

if __name__ == "__main__":
    pass