# coding: utf-8
import rospy #вызов библиотеки rospy
from clover import srv #вызов библиотеки srv
from std_srvs.srv import Trigger #вызов библиотеки Trigger
from cv_bridge import CvBridge #вызов библиотеки CvBridge
from sensor_msgs.msg import Image #вызов библитеки image
import math #вызов библиотеки math
from pyzbar import pyzbar #вызов библиотеки pyzbar

rospy.init_node('flight') 
 
bridge = CvBridge() #задаём значение переменной bridge

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry) #вызов сервиса get_telemetry
navigate = rospy.ServiceProxy('navigate', srv.Navigate) #вызов сервиса navigate
land = rospy.ServiceProxy('land', Trigger) #вызов сервиса land
cord = ['0.345','0.69','0','0','0','0','0'] #выбор начальных координат
Z = 0.9 #задаём переменной Z значение 0.9

def land_wait(): #создаём функцию посадки коптера
	land() 
	while get_telemetry().armed: 
        	rospy.sleep(0.2) 

def qr_check(cord): #создаем функцию сканирование qr кода
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
	a = float(x[2:])
        b = float(y[2:])
	a1 = float(x1[2:])
        b1 = float(y1[2:])
	Xx = float(X[2:])
        Yy = float(Y[2:])
	cord = [a, b, a1, b1, Xx, Yy]
    return cord
	


navigate(x=0, y=0, z=1.5, speed=0.5, frame_id='body', auto_arm=True) #взлёт коптера 
rospy.sleep(7) #адержка

for i in range(4): #условие
    navigate_wait(x=float(cord[0]), y=float(cord[1]), z=1.5, speed = 0.5, frame_id = 'aruco_map')
    if i != 3:
        for j in range(5): 
            navigate_wait(x=float(cord[0] - 0.4), y= float(cord[1] - 0.4), z=Z, speed = 0.5, frame_id = 'aruco_map')
	    navigate_wait(x=float(cord[2] - 0.4), y= float(cord[3] - 0.4), z=Z, speed = 0.5, frame_id = 'aruco_map')
	    navigate_wait(x=float(cord[4]), y= float(cord[5]), z=Z, speed = 0.5, frame_id = 'aruco_map')
            cord = qr_check(cord)
            if pac:
                Z = 1
                pac = False
    	        break
            elif not pac and Z > 0.5:
	            Z -= 0.1
            elif Z <= 0.5:
		        land_wait() 
		        break 
            rospy.sleep(1)
	if Z <= 0.5: 
		break 
print('Column area x=', x,' y=', y,', Navigation area x=', X,', y=', Y,' Order number: ', N,) #вывод данных
navigate(x=0, y=0, z=1.5, speed=0.5, frame_id='aruco_map') #полёт коптера в изначальную точку
	
land_wait() #посадка коптера