#!/usr/bin/env python
import rospy
import cv2
import rospkg, os
import numpy as np
import copy
from cv_bridge import CvBridge
rospack = rospkg.RosPack()



def main():
    rospy.init_node('vision_filter')
    rate = rospy.Rate(1)
    bridge = CvBridge()
    path = rospack.get_path("hg_fusion")
    fileName = '/data/vision_raw.jpg'
    # print(path)
    img = cv2.imread(path+fileName)
    img = cv2.resize(img,(1000,500))
    img_show=copy.copy(img)
    

  
    while not rospy.is_shutdown():
        cv2.imshow('image', img_show)    
        cv2.waitKey(20)
        rate.sleep()
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        gray = np.float32(gray)
        dst = cv2.cornerHarris(gray,10,7,0.1)
        thr = dst.max() * 0.09

        corners = cv2.goodFeaturesToTrack(gray, 400, 0.01, 5)
        corners = np.int0(corners)


        maxX=-999
        maxXy=0
        maxY=-999
        maxYx=0

        minX=999
        minXy=0
        minY=999
        minYx=0    
        for i in corners:
            x, y = i.ravel()
            if (30 < y and y<250) and (300< x and x < 550):
                cv2.circle(img_show, (x,y), 3, 255, -1)
                if x<minX:
                    minX=x
                    minXy=y
                if x>maxX:
                    maxX=x
                    maxXy=y
                if y<minY:
                    minY=y
                    minYx=x
                if y>maxY:
                    maxY=y
                    maxYx=x
        print("({},{}) ,({},{}) ,({},{}) ,({},{}) ".format(minXy,minX,minY,minYx,maxY,maxYx,maxXy,maxX,))


    
    
    
    rospy.spin()


if __name__=='__main__':
    main()
    