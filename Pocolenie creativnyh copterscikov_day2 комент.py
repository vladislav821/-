
import rospy #импортируем библиотеку rospy
import cv2 #импортируем библиотеку cv2
from sensor_msgs.msg import Range #из библиотеки sensor_msgs.msg импортируем файл Range
from clover import srv #из библиотеки sensor_msgs.msg импортируем файл Range
from std_srvs.srv import Trigger #из библиотеки std_srvs.srv импортируем файл Trigger

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry) #вызов сервиса get_telemetry
navigate = rospy.ServiceProxy('navigate', srv.Navigate) #вызов сервиса navigate
land = rospy.ServiceProxy('land', Trigger) #вызов сервиса land

navigate(x=0, y=0, z=1.5, frame_id='body', auto_arm=True) #перелёт квадрокоптера в определенную точку с определённой скоростью

rospy.sleep(4) #задержка 4 сек

navigate(x=0, y=0, z=1.5, frame_id='aruco_map') #перелёт квадрокоптера в определенную точку с определённой скоростью

rospy.sleep(5) #задержка 5 сек

navigate(x=1.2, y=1.2, z=1.5, frame_id='aruco_map') #перелёт квадрокоптера в определенную точку с определённой скоростью
rospy.sleep(5) #задержка 5 сек
print('checking') #вывод сообщения
rospy.sleep(5) #задержка 5 сек
navigate(x=0, y=0.77, z=0, frame_id='body') #перелёт квадрокоптера в определенную точку с определённой скоростью
rospy.sleep(5) #задержка 5 сек
navigate(x=0, y=0, z=-1, frame_id='body') #перелёт квадрокоптера в определенную точку с определённой скоростью
rospy.sleep(5) #задержка 5 сек
print('gruz dostavlen') #вывод сообщения

rospy.sleep(6) #задержка 6 сек
navigate(x=0, y=0, z=1, frame_id='body') #перелёт квадрокоптера в определенную точку с определённой скоростью
rospy.sleep(5) #задержка 5 сек
navigate(x=0, y=0, z=1.5, frame_id='aruco_map') #перелёт квадрокоптера в определенную точку с определённой скоростью

rospy.sleep(5) #задержка 5 сек

land() #посадка квадрокоптера
print('finished') #вывод сообщения