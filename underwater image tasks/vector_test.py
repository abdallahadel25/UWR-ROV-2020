#!/usr/bin/python3

import cv2
import numpy as np


def origin (img):

        def distance (check,hull_ex) :
                dis = 0
                for point in hull_ex:
                        dis = dis + np.sqrt((check[0,0]-point[0,0])**2+(check[0,1]-point[0,1])**2)
                return dis

        height,width,_=img.shape
        img_g = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        _,black = cv2.threshold(img_g,100,255,cv2.THRESH_BINARY)
        black = cv2.morphologyEx(black, cv2.MORPH_OPEN, np.ones((8,8),np.uint8))
        
        _,contours, hierarchy=cv2.findContours(cv2.bitwise_not(black),cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)

        #contours = sorted(contours,key=lambda x:cv2.contourArea(x), reverse=True)
        
        calc_area = lambda x:cv2.boundingRect(x)[2]*cv2.boundingRect(x)[3]
        contours = sorted(contours, key=calc_area, reverse=True)
        
        if len(contours) > 2 :

                hull=cv2.convexHull(contours[0])
                past=0
                corner=hull[0]
                hull_ex_x=[]
                hull_ex_y=[]

                #hull_ex = [ex for ex in hull if ex[0,0]<=5 or ex[0,0]>=width-5 or ex[0,1]<=5 or ex[0,1]>=height-5]

                for i in range(len(hull)) :
                        ex=hull[i]
                        if ex[0,0]<=5 or ex[0,0]>=width-5 :
                                hull_ex_x.append(ex)
                                np.delete(hull,i)
                        if ex[0,1]<=5 or ex[0,1]>=height-5:
                                hull_ex_y.append(ex)
                                np.delete(hull,i)

                for check in range(len(hull)) :
                        dis = distance(hull[check], hull_ex_x) + distance(hull[check], hull_ex_y)
                        if dis>past :
                                past = dis
                                corner = hull[check]

                dummy=0

                if len(hull_ex_x)==0 or len(hull_ex_y)==0 :
                        return   True,corner,contours[1],black,False,0,0,0

                for pt_x in hull_ex_x :
                        for pt_y in hull_ex_y :

                                ba = pt_x[0] - corner[0]
                                bc = pt_y[0] - corner[0]

                                cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
                                angle = np.arccos(cosine_angle)

                                if dummy==0 :
                                        dummy=1
                                        xangle=angle
                                        co_x=pt_x
                                        co_y=pt_y

                                elif angle > xangle :
                                        xangle=angle
                                        co_x=pt_x
                                        co_y=pt_y

                hull_temp=cv2.convexHull(contours[1])
                hull_temp=sorted(hull_temp,key=lambda x:np.sqrt((x[0,0]-corner[0,0])**2+(x[0,1]-corner[0,1])**2))

                return True,corner,contours[1],black,True,co_x,co_y,hull_temp[0]

        else :
                return False,0,0,black,False,0,0,0

def get_prespective(img,black,temp_pt,corner,x,y,height,width):

        pts1 = np.float32([corner[0],y[0],x[0]])
        pts2 = np.float32([[0,height],[0,0],[width,height]])

        M = cv2.getAffineTransform(pts1,pts2)

        dst = cv2.warpAffine(img,M,(width,height))
        dst_b = cv2.warpAffine(black,M,(width,height))

        ones = np.ones(shape=(len(temp_pt), 1))

        points_ones = np.hstack([temp_pt, ones])

        rotated_point = M.dot(points_ones.T)

        dst= cv2.circle(dst,(rotated_point[0],rotated_point[1]),5,(255,0,0),-1)

        return dst,dst_b,rotated_point

def square (black):
        
        height,width=black.shape
        _,contours, hierarchy=cv2.findContours(black,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
        contours = sorted(contours, key=lambda x:np.sqrt((cv2.minEnclosingCircle(x)[0][0])**2+(cv2.minEnclosingCircle(x)[0][1]-height)**2), reverse=False)
        points=cv2.boundingRect(contours[0])

        #x,y,w,h
        return points

def count(dst_b,square_p,rotated_point):

        count_x=dst_b[square_p[1]+int(square_p[3]*0.3):square_p[1]+int(square_p[3]*0.7),square_p[0]:int(rotated_point[0])-int(square_p[2]*0.4)]
        count_y=dst_b[int(rotated_point[1])+int(square_p[3]*0.4):square_p[1]+square_p[3],square_p[0]+int(square_p[2]*0.3):square_p[0]+int(square_p[2]*0.7)]

        _,contour_x, hierarchy=cv2.findContours(count_x,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
        _,contour_y, hierarchy=cv2.findContours(count_y,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)

        return len(contour_x),len(contour_y)

def vector(img):

        #ret, img = cap.read()
        #img = cv2.imread("test/test-5.jpg")
        
        height,width,_=img.shape

        vector_length = -1
        vector_angle = -1

        step_x = -1
        step_y = -1

        check,corner,temp,black,check_co,x,y,temp_pt=origin(img)
        print(temp_pt)

        if check :

                if check_co :

                        dst,dst_b,rotated_point=get_prespective(img,black,temp_pt,corner,x,y,height,width)

                        square_p=square(dst_b)
                        
                        step_x,step_y=count(dst_b,square_p,rotated_point)

                        #print(step_x,"   ",step_y)
                        
                        co_x = (step_x+7.5)*1.5
                        co_y = (step_y+7.5)*1.5

                        vector_length = np.sqrt(co_x**2+co_y**2)
                        vector_angle = np.degrees(np.arctan(co_y/co_x))

                        print("V = ({},{})".format(vector_length,vector_angle))

                        img= cv2.line(img,(corner[0,0],corner[0,1]),(x[0,0],x[0,1]),(0,0,255),3)
                        img= cv2.line(img,(corner[0,0],corner[0,1]),(y[0,0],y[0,1]),(0,0,255),3)
                
                img=cv2.drawContours(img,temp,-1,(0,255,0),3)
                img= cv2.circle(img,(corner[0,0],corner[0,1]),5,(0,255,0),-1)
                img= cv2.circle(img,(temp_pt[0,0],temp_pt[0,1]),5,(255,0,0),-1)
        
        return (img,vector_length,vector_angle,step_x,step_y)

#cap = cv2.VideoCapture('rtsp://admin:Mahmoud@1205@192.168.1.64:554/Streaming/Channels/101')
#cap=cv2.VideoCapture('rtsp://admin:admin@192.168.1.11:554/cam/realmonitor?channel=1&subtype=0')
#cap = cv2.VideoCapture('http://192.168.1.4:4747/video')
#cap=cv2.VideoCapture(1)

if __name__ == "__main__":
    
        test = cv2.imread("test/test-5.jpg")
        #while True :
        res = vector(test)[0]
        cv2.imshow("img",res)
        cv2.waitKey(0)
        cv2.destroyAllWindows()