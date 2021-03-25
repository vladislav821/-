# coding: utf-8
import rospy #вызов библиотеки rospy
import math #вызов библиотеки math
from math import sqrt #вызов библиотеки math
import csv #вызов библиотеки csv
from rosgraph_msgs.msg import Clock #вызов библиотеки Clock

import cv2 as cv #вызов библиотеки cv
import numpy as np #вызов библиотеки np
from sensor_msgs.msg import Image #вызов библиотеки Image
from cv_bridge import CvBridge #вызов библиотеки CvBridge
from pyzbar import pyzbar #вызов библиотеки pyzbar
from sensor_msgs.msg import Range #вызов библиотеки Range
from geometry_msgs.msg import PoseStamped #вызов библиотеки PoseStamped

from clover import srv #вызов библиотеки srv
from std_srvs.srv import Trigger #вызов библиотеки Trigger
from clover.srv import SetLEDEffect #вызов библиотеки SetLEDEffect

import tf2_ros #вызов библиотеки tf2_ros
import tf2_geometry_msgs #вызов библиотеки tf2_geometry_msgs

rospy.init_node('flight') 
 
bridge = CvBridge() #задаём значение переменной bridge

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry) #вызов сервиса get_telemetry
navigate = rospy.ServiceProxy('navigate', srv.Navigate) #вызов сервиса navigate
land = rospy.ServiceProxy('land', Trigger) #вызов сервиса land
cord = ['0','0'] #выбор начальных координат (0)
cord1 = ['0','0'] #выбор начальных координат (1)
cord2 = ['0','0'] #выбор начальных координат (2)
cord3 = ['0','0'] #выбор начальных координат (3)
Z = 1 #задаём переменной Z значение 1


def land_wait(): #выполняет посадку коптера
	land() #посадка квадрокоптера
	while get_telemetry().armed: #условие
        	rospy.sleep(0.2) #задержка 0.2 сек

def color_detect(cv_image): #функция определения цвета
    color = {'green': [40, 100, 100, 80, 255, 255, [0, 255, 0]], 'red_1': [170, 100, 100, 179, 255, 255, [0, 0, 255]], 'red_2': [0, 100, 100, 10, 255, 255, [0, 0, 255]], 'yellow': [20, 100, 100, 40, 255, 255, [0, 255, 255]], 'blue': [110, 100, 100, 130, 255, 255, [255, 0, 0]]}
    colors_name = ['green', 'red_1', 'red_2', 'yellow', 'blue']
    
    ret = []

    for name in colors_name:
        hsv_param = color[name]
        hsv_min = np.array((hsv_param[0], hsv_param[1], hsv_param[2]), np.uint8)
        hsv_max = np.array((hsv_param[3], hsv_param[4], hsv_param[5]), np.uint8)

        hsv = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)
        thresh = cv.inRange(hsv, hsv_min, hsv_max)
        new_image, contours0, hierarchy = cv.findContours(thresh.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours0:
            rect = cv.minAreaRect(cnt)
            box = cv.boxPoints(rect)
            box = np.int0(box)

            thresh_new = cv.GaussianBlur(thresh, (5, 5), 2)
            rows = thresh_new.shape[0]
            circles = []
            circles = cv.HoughCircles(thresh_new, cv.HOUGH_GRADIENT, 1, rows / 8,
                                    param1=100, param2=30,
                                    minRadius=1, maxRadius=50)

            if circles is not None:
                circles = np.uint16(np.around(circles))
                for i in circles[0, :]:
                    center = [i[0], i[1]]
                    ret.append([center, 'circle', name])

            if sqrt((box[0][0] - box[2][0])**2 + (box[0][1] - box[2][1])**2) > 20:
                
                min_x = box[:, 0].min()
                max_x = box[:, 0].max()
                min_y = box[:, 1].min()
                max_y = box[:, 1].max()

                new_min_y = min_y-20 if min_y-20 >= 0 else 0
                new_max_y = max_y+20 if max_y+20 >= 0 else 0
                new_min_x = min_x-20 if min_x-20 >= 0 else 0
                new_max_x = max_x+20 if max_x+20 >= 0 else 0

                thresh_new = thresh[new_min_y:new_max_y, new_min_x:new_max_x]

                moments = cv.moments(thresh_new, 1)
                dM01 = moments['m01']
                dM10 = moments['m10']
                dArea = moments['m00']

                x = int(dM10 / dArea) + new_min_x
                y = int(dM01 / dArea) + new_min_y

                k = 0
                try:
                    for i in circles[0, :]:
                        if  abs(i[0] - x) < 10 and abs(i[1] - y) < 10:
                            k += 1
                except TypeError:
                    k == 0

                if k == 0:
                    ret.append([[x, y], 'rectangle', name])

    return ret


def contour(cv_image): #функция определения контура
    color = {'green': [40, 100, 100, 80, 255, 255, [0, 255, 0]], 'red_1': [170, 100, 100, 179, 255, 255, [0, 0, 255]], 'red_2': [0, 100, 100, 10, 255, 255, [0, 0, 255]], 'yellow': [20, 100, 100, 40, 255, 255, [0, 255, 255]], 'blue': [110, 100, 100, 130, 255, 255, [255, 0, 0]]}
    colors_name = ['green', 'red_1', 'red_2', 'yellow', 'blue']

    for name in colors_name:
        hsv_param = color[name]
        hsv_min = np.array((hsv_param[0], hsv_param[1], hsv_param[2]), np.uint8)
        hsv_max = np.array((hsv_param[3], hsv_param[4], hsv_param[5]), np.uint8)


        hsv = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)
        thresh = cv.inRange(hsv, hsv_min, hsv_max)
        new_image, contours0, hierarchy = cv.findContours(thresh.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours0:
            rect = cv.minAreaRect(cnt)
            box = cv.boxPoints(rect)
            box = np.int0(box)

            thresh_new = cv.GaussianBlur(thresh, (5, 5), 2)
            rows = thresh_new.shape[0]
            circles = cv.HoughCircles(thresh_new, cv.HOUGH_GRADIENT, 1, rows / 8,
                                    param1=100, param2=30,
                                    minRadius=1, maxRadius=50)

            if circles is not None:
                circles = np.uint16(np.around(circles))
                for i in circles[0, :]:
                    x_center_circle = i[0]
                    y_center_circle = i[1]
                    radius = i[2]
            else:
                x_center_circle = -1
                y_center_circle = -1
                radius = -1

            if sqrt((box[0][0] - box[2][0])**2 + (box[0][1] - box[2][1])**2) > 20:
                
                min_x = box[:, 0].min()
                max_x = box[:, 0].max()
                min_y = box[:, 1].min()
                max_y = box[:, 1].max()

                new_min_y = min_y-20 if min_y-20 >= 0 else 0
                new_max_y = max_y+20 if max_y+20 >= 0 else 0
                new_min_x = min_x-20 if min_x-20 >= 0 else 0
                new_max_x = max_x+20 if max_x+20 >= 0 else 0

                thresh_new = thresh[new_min_y:new_max_y, new_min_x:new_max_x]

                moments = cv.moments(thresh_new, 1)
                dM01 = moments['m01']
                dM10 = moments['m10']
                dArea = moments['m00']

                x = int(dM10 / dArea) + new_min_x
                y = int(dM01 / dArea) + new_min_y

                if x_center_circle != -1 and y_center_circle != -1 and abs(x_center_circle - x) >= 10 and abs(y_center_circle - y) >= 10:
                    cv.drawContours(cv_image, [box], 0, (hsv_param[6][0], hsv_param[6][1], hsv_param[6][2]), 2)
                if x_center_circle != -1 and y_center_circle != -1 and radius != -1:
                    cv.circle(cv_image, (x_center_circle, y_center_circle), radius, (hsv_param[6][0], hsv_param[6][1], hsv_param[6][2]), 2)

                cv.circle(cv_image, (x, y), 5, (0, 0, 0), -1)
                cv.rectangle(cv_image, (x, y - 18), (x + 50, y), (255, 255, 255), -1)
                cv.rectangle(cv_image, (x, y - 18), (x + 50, y), (0, 0, 0), 1)
                if name == 'red_1' or name == 'red_2':
                    name_color = 'red'
                else:
                    name_color = name
                cv.putText(cv_image, name_color, (x + 3, y - 5), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

    image_pub.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))

def qr_check(cord): #функция считывания qr-кодов
    img = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')  
    p = pyzbar.decode(img)
    global pac 
    pac = False
    print('reading') 
    if len(p) != 0:
        pac = True 
        for p in p:
	    	pdata = p.data.encode("utf-8") 
        x, y, x1, y1, X, Y, N = map(str, pdata.split()) 
	Xx = float(x[2:])
        Yy = float(y[2:])
	Xx1 = float(x1[2:])
        Yy1 = float(y1[2:])
	XX = float(X[2:])
        YY = float(Y[2:])
	cord1 = [Xx, Yy]
	cord2 = [Xx1, Yy1]
	cord3 = [XX, YY]
    
navigate(x=0, y=0, z=1.5, speed=0.5, frame_id='body', auto_arm=True) #взлёт квадрокоптера
rospy.sleep(7)  #задержка
navigate(x=0.4, y=0.8, z=Z, speed = 0.5, frame_id = 'aruco_map') #полёт квадрокоптера в точку с qr-одов
for j in range(5): 
    cord1 = qr_check(cord)
    cord2 = qr_check(cord)
    cord3 = qr_check(cord)
    if pac:
        Z = 1
        pac = False
    	break
    elif not pac and Z > 0.5:
            Z -= 0.1
    elif Z <= 0.5:
		land_wait() 
	        break 
    rospy.sleep(3)

def video_reader(): #функция записи видео
    cam = cv2.VideoCapture(0) 
    detector = cv2.QRCodeDetector() 
    while True:
        _, img = cam.read()
        data, bbox, _ = detector.detectAndDecode(img)
        if data:
            print("QR Code detected-->", data)
        cv2.imshow("img", img)    
        if cv2.waitKey(1) == ord("Q"):
            break
    cam.release()
    cv2.destroyAllWindows()

video_reader()


	
land_wait()	#посадка квадрокоптера