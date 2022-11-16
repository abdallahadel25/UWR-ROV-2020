#!/usr/bin/python3

import cv2
import numpy as np

def detect(img):

    h,w,_ = img.shape
    area = h*w

    img = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
    res = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY_INV,11,4)
    res = cv2.dilate(res, np.ones((2,2),np.uint8),iterations = 1)
    res = cv2.morphologyEx(res, cv2.MORPH_OPEN, np.ones((2,2),np.uint8))
    cv2.imshow("res",res)
    contours, hierarchy=cv2.findContours(res,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
    
    if contours :
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        return (True,contours[0])
    else :
        return(False,0)

def compare (shape,temp):
    
    temp = cv2.cvtColor(temp,cv2.COLOR_BGR2GRAY)
    _,temp = cv2.threshold(temp,127,255,cv2.THRESH_BINARY)
    contours, hierarchy=cv2.findContours(temp,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)

    return (cv2.matchShapes(shape,contours[0],1,0.0))

#cap = cv2.VideoCapture(0)
#cap=cv2.VideoCapture('rtsp://admin:admin@192.168.1.11:554/cam/realmonitor?channel=1&subtype=0')
#cap = cv2.VideoCapture('http://192.168.1.4:4747/video')


def shape_setup():

    circle = cv2.imread("test/circle-temp.jpg")
    triangle = cv2.imread("test/triangle-temp.jpg")
    square = cv2.imread("test/square-temp.jpg")
    star = cv2.imread("test/star-temp.jpg")
    rectangle = cv2.imread("test/rectangle-temp.jpg")

    templets = {'circle':[circle,"2nd Century BC"],'trinagle':[triangle,"Triangle"],'square':[square,"4th Century BC"],'star':[star,"5th Century BC"],'rectangle':[rectangle,"6th Century BC"]}

    return templets

def shape(img,templets):

    #ret, img = cap.read()
    #img = cv2.imread("test/real-3.jpg")

    #img=img[:-50,:]

    check,shape = detect (img)
    
    if check :

        value = compare(shape,templets['circle'][0])
        time=templets['circle'][1] 
        
        for test in templets :
            test_v = compare(shape,templets[test][0])
            if test_v < value :
                value = test_v
                time = templets[test][1]

        if value < 0.1 :
            
            box=cv2.boundingRect(shape)
            img = cv2.rectangle(img,(box[0],box[1]),(box[0]+box[2],box[1]+box[3]),(0,255,0),2)
            img = cv2.putText(img, time, (box[0],box[1]-10), cv2.FONT_HERSHEY_SIMPLEX , 0.8 , (255,0,0), 2, cv2.LINE_AA) 
    

    return (img)

if __name__ == "__main__":

    templets = shape_setup()
    test=cv2.imread("test/real.jpg")
    #while True :
    img = shape(test,templets)
    cv2.imshow("img",img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
