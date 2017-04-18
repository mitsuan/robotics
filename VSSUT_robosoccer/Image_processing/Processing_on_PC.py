import cv2
import sys
import serial as ser
import numpy as np
import time
from math import * 

def nothing(x):
    pass

#Xbee object
xbee=ser.Serial('COM4',9600);

cam = cv2.VideoCapture(1)
ret,img = cam.read()

cv2.namedWindow('PID_W');
cv2.createTrackbar('KP','PID_W',0,100,nothing)
cv2.createTrackbar('KI','PID_W',0,100,nothing)
cv2.createTrackbar('KD','PID_W',0,100,nothing)

cv2.namedWindow('PID_D');
cv2.createTrackbar('KP','PID_D',0,100,nothing)
cv2.createTrackbar('KI','PID_D',0,100,nothing)
cv2.createTrackbar('KD','PID_D',0,100,nothing)

cv2.setTrackbarPos('KP','PID_W',33)
cv2.setTrackbarPos('KP','PID_D',13)

def track(hsv,lower,upper):

      global img
      
      mask = cv2.inRange(hsv, lower, upper)      

      erodelement = np.ones((5,5),np.uint8)
      dilatelement = np.ones((8,8),np.uint8)

      erosion = cv2.erode(mask,erodelement,iterations = 1)
      dilation = cv2.dilate(mask,dilatelement,iterations = 1)

      dilation = cv2.dilate(mask,dilatelement,iterations = 1)
      erosion = cv2.erode(mask,erodelement,iterations = 1)

      contour, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

      t = int(0)
      max_area = int(0)
      cnt = int(0)
      x = int(0)
      y = int(0)
      
      for cnt in range(len(contour)):
            area = cv2.contourArea(contour[cnt])
            if(area>max_area):
                max_area = area
                t = cnt

      if(len(contour)>0):
          (x,y),radius = cv2.minEnclosingCircle(contour[t])
          center = (int(x),int(y))
          radius = int(radius)
          cv2.drawContours(img,contour[t],0,(0,0,255),-1)   # draw contours in green color
          cv2.circle(img,center,radius,(0,0,0),2)

      return [x,y]

    
prev_theta = float(0.0)
w = float(0.0)
p = float(0.0)
i = float(0.0)
d = float(0.0)
def pid_W(theta):
    global prev_theta,w,p,i,d
    kp = cv2.getTrackbarPos('KP','PID_W')/10
    ki = cv2.getTrackbarPos('KI','PID_W')/10
    kd = cv2.getTrackbarPos('KD','PID_W')/10
    p = theta
    d = theta - prev_theta
    i = i + theta
    if(i>200):
        i = 200
    if(i<-200):
        i = -200
    w = kp * p + ki * i + kd * d
    if(w > 300):
        w = 300
    if(w < -300):
        w = -300
    pp = "{:.3f}".format(p)
    ii = "{:.3f}".format(i)
    dd = "{:.3f}".format(d)
    ww = "{:.3f}".format(w)
    prev_theta = theta
    #print(pp,' , ',ii,' , ',dd,' , ',ww)
    return int(float(ww))


prev_dist = float(0.0)
v = float(0.0)
_p = float(0.0)
_i = float(0.0)
_d = float(0.0)
def pid_D(dist):
    global v,_p,_i,_d,prev_dist
    kp = cv2.getTrackbarPos('KP','PID_D')/10
    ki = cv2.getTrackbarPos('KI','PID_D')/10
    kd = cv2.getTrackbarPos('KD','PID_D')/10
    _p = dist
    _d = dist - prev_dist
    _i = _i + dist
    if(_i > 200):
        _i = 200
    if(_i < -200):
        _i = -200
    v = _p * kp + _i * ki + _d * kd
    if( v > 150):
        v = 150
    pp = "{:.3f}".format(_p)
    ii = "{:.3f}".format(_i)
    dd = "{:.3f}".format(_d)
    vv = "{:.3f}".format(v)
    #print(pp,' , ',ii,' , ',dd,' , ',vv)
    prev_dist = dist
    return int(float(vv))

# Output Image to display
output = img

while(1):
      ret,img = cam.read()
      t1=time.time()
      '''
      if img is not None:
          img = img[15:455, 0:300]
      '''    
      hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

      #for color Green      
      g_lower = np.array([73, 152, 129])
      g_upper = np.array([83, 255, 255])

      g_cX = 0
      g_cY = 0
            
      [g_cX,g_cY] = track(hsv,g_lower,g_upper)

      #g_cY = -g_cY
      
      #for color Maroon
      m_lower = np.array([92, 85, 74])
      m_upper = np.array([195, 255, 166])

      m_cX = 0
      m_cY = 0

      [m_cX,m_cY] = track(hsv,m_lower,m_upper)

      #for color Orange
      o_lower = np.array([160, 83, 0])
      o_upper = np.array([188, 186, 255])

      o_cX = 0
      o_cY = 0

      [o_cX,o_cY] = track(hsv,o_lower,o_upper)

      #draw line between green and maroon circle
      cv2.line(img,(int(m_cX),int(m_cY)),(int(g_cX),int(g_cY)),(0,0,255),1)

      #center of line
      center_x = (m_cX + g_cX)/2
      center_y = (m_cY + g_cY)/2
      cv2.circle(img,(int(center_x),int(center_y)),2,(0,0,0),2)

      #draw line from center to ball
      cv2.line(img,(int(center_x),int(center_y)),(int(o_cX),int(o_cY)),(0,0,255),1)

      #finding angle in radians     
      theta_1 = atan2(g_cY-center_y , g_cX-center_x) - atan2(o_cY-center_y , o_cX-center_x)
      if (theta_1 < -pi):
          theta_1 += pi*2
      if (theta_1 > pi):
          theta_1 -= pi*2

      dist = ((o_cY-center_y)**2 + (o_cX-center_x)**2)**0.5
      v_m = float(0.0)
      if(dist > 50):
        v_m = pid_D(dist-50)

      theta_2=theta_1 + (pi/2)
      #v_m=(3**0.5)*255
      x=v_m*cos(theta_2)
      y=v_m*sin(theta_2)
      w=pid_W(degrees(theta_1))

      print(str(x),' , ',str(y),' , ',str(w))
      xbee.write(str(x)+','+str(-y)+','+str(-w))
      '''''
      fourcc = cv2.VideoWriter_fourcc(*'XVID')
      out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640, 480))
      out.write(img)
      '''
      cv2.imshow('out',img)
      #print(time.time()-t1)
      k = cv2.waitKey(33) & 0xFF
      if k == 27:
          xbee.write(str(0) + ',' + str(0) + ',' + str(0))
          break
cam.release()
cv2.destroyAllWindows()
