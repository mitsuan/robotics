import cv2
import sys
import serial as ser
import numpy as np
import time

def nothing(x):
    pass

cam = cv2.VideoCapture(0)
ret,img = cam.read()

def track(hsv,lower,upper):

      global img
      
      mask = cv2.inRange(hsv, lower, upper)      

      erodelement = np.ones((5,5),np.uint8)
      dilatelement = np.ones((8,8),np.uint8)

      erosion = cv2.erode(mask,erodelement,iterations = 1)
      dilation = cv2.dilate(mask,dilatelement,iterations = 1)

      dilation = cv2.dilate(mask,dilatelement,iterations = 1)
      erosion = cv2.erode(mask,erodelement,iterations = 1)

      contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

      return contours
    
def draw_rect(contour):
    x = 0
    y = 0
    global img
    for cnt in contour:
        epsilon = 0.1*cv2.arcLength(cnt,True)
        approx = cv2.approxPolyDP(cnt,epsilon,True)
        M = cv2.moments(cnt)
        if(M['m00']!=0):
              x = int(M["m10"] / M["m00"])
              y = int(M["m01"] / M["m00"])
        if(len(approx)==4):
              cv2.drawContours(img,[approx],-1,(0,255,0),-1) 
              cv2.circle(img, (x, y), 3, (255, 255, 255), -1)    
    return [x,y]


cv2.namedWindow('tb')

# create trackbars for color change
cv2.createTrackbar('HMin','tb',0,179,nothing) # Hue is from 0-179 for Opencv
cv2.createTrackbar('HMax','tb',0,179,nothing)
cv2.createTrackbar('SMin','tb',0,255,nothing)
cv2.createTrackbar('SMax','tb',0,255,nothing)
cv2.createTrackbar('VMin','tb',0,255,nothing)
cv2.createTrackbar('VMax','tb',0,255,nothing)

# Set default value for MAX HSV trackbars.
cv2.setTrackbarPos('HMin', 'tb', 0)
cv2.setTrackbarPos('SMin', 'tb', 100)
cv2.setTrackbarPos('VMin', 'tb', 255)

cv2.setTrackbarPos('HMax', 'tb', 186)
cv2.setTrackbarPos('SMax', 'tb', 183)
cv2.setTrackbarPos('VMax', 'tb', 255)

# Output Image to display
output = img

while(1):
      ret,img = cam.read()
      t1=time.time()
      if img is not None:
          img = img[15:455, 0:300]
      hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
      
      # get current positions of all trackbars
      hMin = cv2.getTrackbarPos('HMin','tb')
      sMin = cv2.getTrackbarPos('SMin','tb')
      vMin = cv2.getTrackbarPos('VMin','tb')

      hMax = cv2.getTrackbarPos('HMax','tb')
      sMax = cv2.getTrackbarPos('SMax','tb')
      vMax = cv2.getTrackbarPos('VMax','tb')
      
      #for color Green
      g_lower = np.array([73, 152, 129])
      g_upper = np.array([83, 255, 255])

      g_cX = 0
      g_cY = 0
      
      contour = track(hsv,g_lower,g_upper)

      for cnt in contour:
            (g_cX,g_cY),radius = cv2.minEnclosingCircle(cnt)
            center = (int(g_cX),int(g_cY))
            radius = int(radius)
            cv2.drawContours(img,[cnt],0,(0,0,255),-1)   # draw contours in green color
            cv2.circle(img,center,radius,(0,0,0),2)
        
      #[g_cX,g_cY] = draw_rect(contour)
      cv2.imshow('green',img)
      #for color Blue
      b_lower = np.array([103, 90, 229])
      b_upper = np.array([128, 123, 255])

      b_cX = 0
      b_cY = 0

      contour = track(hsv,b_lower,b_upper)

      for cnt in contour:
            (b_cX,b_cY),radius = cv2.minEnclosingCircle(cnt)
            center = (int(b_cX),int(b_cY))
            radius = int(radius)
            cv2.drawContours(img,[cnt],0,(0,0,255),-1)   # draw contours in green color
            cv2.circle(img,center,radius,(0,0,0),2)

      #[b_cX,b_cY] = draw_rect(contour)
      cv2.imshow('blue',img)
      #for color Orange
      o_lower = np.array([hMin, sMin, vMin])
      o_upper = np.array([hMax, sMax, vMax])

      o_cX = 0
      o_cY = 0

      contour = track(hsv,o_lower,o_upper)
      
      for cnt in contour:
            (x,y),radius = cv2.minEnclosingCircle(cnt)
            center = (int(x),int(y))
            radius = int(radius)
            cv2.drawContours(img,[cnt],0,(0,0,255),2)   # draw contours in green color
            cv2.circle(img,center,radius,(0,255,0),2)

      #cv2.line(img,(b_cX,b_cY),(g_cX,g_cY),(0,0,255),3)
      cv2.imshow('out',img)
      #print(time.time()-t1)
      k = cv2.waitKey(33) & 0xFF
      if k == 27:
            break      
cam.release()
cv2.destroyAllWindows()
