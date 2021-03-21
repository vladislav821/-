# coding: utf8
import rospy
import math
from math import sqrt

import csv
from rosgraph_msgs.msg import Clock

import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pyzbar import pyzbar
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped

from clover import srv
from std_srvs.srv import Trigger
from clover.srv import SetLEDEffect

import tf2_ros
import tf2_geometry_msgs

from aruco_pose.msg import MarkerArray

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)

bridge = CvBridge()

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)



# Correct landing
def land_wait():
    land()
    while get_telemetry().armed:
        rospy.sleep(0.2)
    print('Landing is successfully performed')
    print('Program execution is finished')

# Flying to a point and waiting for the end of the flight
def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

# Return to point (0; 0)
def go_to_home():
    navigate_wait(x=0, y=0, z=1.5, speed=1.5, frame_id='aruco_map')
    navigate_wait(x=0, y=0, z=0.8, speed=1.5, frame_id='aruco_map')
    land_wait()


def color_detect(cv_image):
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


def contour(cv_image):
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


def image_callback_cm(data):
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
    contour(cv_image)


def color_inf():
    color = {'green': [40, 100, 100, 80, 255, 255, [0, 255, 0]], 'red_1': [170, 100, 100, 179, 255, 255, [0, 0, 255]], 'red_2': [0, 100, 100, 10, 255, 255, [0, 0, 255]], 'yellow': [20, 100, 100, 40, 255, 255, [0, 255, 255]], 'blue': [110, 100, 100, 130, 255, 255, [255, 0, 0]]}
    img = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
    # img = img[80:160, 100:220]
    inf_metka = color_detect(img)

    np_inf_metka = np.array(inf_metka)    
    np_inf_metka_centres = np_inf_metka[:, 0]
    dist = 1000
    number_of_cm = 0
    counter = 0
    for centre in np_inf_metka_centres:
        dist_new = sqrt((160 - centre[0])**2 + (120 - centre[1])**2)
        if dist_new < dist:
            dist = dist_new
            number_of_cm = counter
        counter += 1

    inf_metka_new = inf_metka[number_of_cm]
    color_name = inf_metka_new[2]
    diap = color[color_name]
    diap[0:6]
    shape = inf_metka_new[1]
    if color_name == 'red_1' or color_name == 'red_2':
        name_of_cm = 'red'
    else:
        name_of_cm = color_name
    inf_metka = [name_of_cm, diap, shape]


    bgr = color[color_name]
    bgr = bgr[6]
    b, g, r = bgr[0], bgr[1], bgr[2]
    set_effect(effect='blink', r=r, g=g, b=b)
    rospy.sleep(2)
    set_effect(effect='fill', r=255, g=0, b=255)

    return inf_metka

def map_cm():
    aruco_detect_markers = rospy.wait_for_message('aruco_detect/markers', MarkerArray)
    aruco_detect_marker_1 = aruco_detect_markers.markers[0]
    aruco_detect_marker_2 = aruco_detect_markers.markers[1]
    id_aruco_detect_marker_1 = aruco_detect_marker_1.id
    id_aruco_detect_marker_2 = aruco_detect_marker_2.id

    aruco_map_markers = rospy.wait_for_message('aruco_map/markers', MarkerArray)
    aruco_map_markers = aruco_map_markers.markers
    for marker in aruco_map_markers:
        if marker.id == id_aruco_detect_marker_1:
            aruco_map_marker_1 = marker
        if marker.id == id_aruco_detect_marker_2:
            aruco_map_marker_2 = marker

    x_center_of_aruco_detect_marker_1 = (aruco_detect_marker_1.c1.x + aruco_detect_marker_1.c3.x) / 2
    y_center_of_aruco_detect_marker_1 = (aruco_detect_marker_1.c1.y + aruco_detect_marker_1.c3.y) / 2
    x_center_of_aruco_detect_marker_2 = (aruco_detect_marker_2.c1.x + aruco_detect_marker_2.c3.x) / 2
    y_center_of_aruco_detect_marker_2 = (aruco_detect_marker_2.c1.y + aruco_detect_marker_2.c3.y) / 2
    distance_between_aruco_detect_markers = math.sqrt((x_center_of_aruco_detect_marker_1 - x_center_of_aruco_detect_marker_2)**2 + (y_center_of_aruco_detect_marker_1 - y_center_of_aruco_detect_marker_2)**2)

    x_aruco_map_marker_1 = aruco_map_marker_1.pose.position.x
    y_aruco_map_marker_1 = aruco_map_marker_1.pose.position.y
    x_aruco_map_marker_2 = aruco_map_marker_2.pose.position.x
    y_aruco_map_marker_2 = aruco_map_marker_2.pose.position.y
    distance_between_aruco_map_markers = math.sqrt((x_aruco_map_marker_1 - x_aruco_map_marker_2)**2 + (y_aruco_map_marker_1 - y_aruco_map_marker_2)**2)

    scale = distance_between_aruco_map_markers / distance_between_aruco_detect_markers

    map_coordinates_of_the_copter = rospy.wait_for_message('mavros/local_position/pose', PoseStamped)
    map_coordinates_of_the_copter = tf_buffer.transform(map_coordinates_of_the_copter, 'aruco_map', rospy.Duration(0.2))

    x_map_copter = map_coordinates_of_the_copter.pose.position.x
    y_map_copter = map_coordinates_of_the_copter.pose.position.y

    x_img_copter = 160
    y_img_copter = 120

    cv_image = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
    list_of_the_cm = color_detect(cv_image)
    map_of_the_cm = []
    for cm in list_of_the_cm:
        x_img_cm = cm[0][0]
        y_img_cm = cm[0][1]

        if x_img_cm > x_img_copter:
            y_map_cm = y_map_copter - abs(x_img_cm - x_img_copter)*scale
        else:
            y_map_cm = y_map_copter + abs(x_img_cm - x_img_copter)*scale
        
        if y_img_cm > y_img_copter:
            x_map_cm = x_map_copter - abs(y_img_cm - y_img_copter)*scale
        else:
            x_map_cm = x_map_copter + abs(y_img_cm - y_img_copter)*scale

        x_map_cm = round(x_map_cm, 4)
        y_map_cm = round(y_map_cm, 4)
        new_cm_to_map = [[x_map_cm, y_map_cm], cm[1], cm[2]]
        map_of_the_cm.append(new_cm_to_map)
    
    return map_of_the_cm

popit = 10

#Функция нахождения из данной строки
#числа в float()

#Read QR code 1 times
def check_for_QR():
    global QR_data
    flag_find_QR = False
    cv_image = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
    barcodes = pyzbar.decode(cv_image)
    for barcode in barcodes:
        QR_data = barcode.data.decode("utf-8")
        flag_find_QR = True
    return flag_find_QR

#Read QR (несколько снижений, и считывание QR на каждой из них)
def check_read(xc, yc):
    flag_of_result = False
    navigate_wait(x=xc, y=yc, z=3.4, speed=1.0, frame_id='aruco_map')
    #Заходы(снижения)
    for height in heights:
        navigate_wait(x=xc, y=yc, z=height, speed=1.0, frame_id='aruco_map')
        rospy.sleep(2)
        #Считывание QR несколько раз
        for i in range(popit):
            # Check QR code read
            flag_find_QR = check_for_QR()
            if flag_find_QR == True:
                flag_of_result = True
                break
        if flag_of_result == True:
            break
    return flag_of_result
    




    



qrCodes = [[0.530742, 0.058797], [8.53237, 4.49862], [5.48956, 8.88549], [4.52521, 4.80307], [2.49273, 0.910025], [7.51784, 1.70644], [1.479290, 8.047910], [0.017382, 3.504660]]

#Max height
z_decline = 1.1
#Minimum height
z_minimum = 0.5
#Number of attempts to read the QR code
attempts = 7
#Change height for a new attempt
z_attempts = (z_decline - z_minimum) / attempts

#Creating a list of height values for reading QR codes
heights = []
for i in range(attempts + 1):
    new_height = z_decline - i * z_attempts
    heights.append(new_height)

#Up on body
navigate_wait(x=0, y=0, z=2, speed=1.0, frame_id='body', auto_arm=True)
rospy.sleep(1)
#Binding to aruco_map
navigate_wait(x=0, y=0, z=2, speed=1.0, frame_id='aruco_map')

rospy.sleep(1)
# Process of reading 3 QR codes and flying to points
for i in range(len(qrCodes)):
    #Flying to the QR code
    X_Y = qrCodes[i]
    flag_of_result = check_read(X_Y[0], X_Y[1])
    
    #Экстренная посадка, если по прошествию всех попыток QR кода не нашлось
    #(сделано для того, что бы коптер не висел в воздухе и его не пришлось перехватывать)
    if flag_of_result == False:
        print('Didnt find any QR')
        land_wait()
        break
    else:
        #Info of decoded QR
        print(QR_data)
    navigate_wait(x=X_Y[0], y=X_Y[1], z=2, speed=1.0, frame_id='aruco_map')
    map_of_the_cm = map_cm()
    print map_of_the_cm
    navigate_wait(x=X_Y[0], y=X_Y[1], z=3.4, speed=1.0, frame_id='aruco_map')
# The rise




#navigate_wait(x=map_of_the_cm[0][0][0], y=map_of_the_cm[0][0][1], z=1, speed=1, frame_id='aruco_map')
rospy.sleep(5)
land_wait()