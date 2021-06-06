#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

from math import floor
from numpy import inf
import time

imu_y = 0

def callback_imu(msg):
   #print("Imu Seq: [%d]" % msg.header.seq);
   #print("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]" % 
   #                          ( msg.orientation.x,
   #                           msg.orientation.y,
   #                           msg.orientation.z,
   #                           msg.orientation.w )
   global imu_y

   imu_y = msg.orientation.y
   


# get the laser messages
def callback_laser(msg):   
   laser_raw = msg.ranges

   #print(msg.angle_min)        # start angle of the scan [rad]
   #print(msg.angle_max)        # end angle of the scan [rad]
   #print(msg.angle_increment)  # angular distance between measurements [rad]
   #print(msg.time_increment)   # time between measurements [seconds]
   #print(msg.scan_time)        # time between scans [seconds]
   #print(msg.range_min)        # minimum range value [m]
   #print(msg.range_max)        # maximum range value [m]

   laser_float = [float(r) for r in laser_raw]

   rf_sensor = min(laser_float[(int(30*2)-1):(int(60*2)-1)])
   f_sensor = min(laser_float[(int(61*2)-1):(int(120*2)-1)])
   lf_sensor = min(laser_float[(int(121*2)-1):(int(150*2)-1)])

   print('\n\nRight-Front: ' + str(rf_sensor) )
   print('Front: ' + str(f_sensor) )
   print('Left-Front: ' + str(lf_sensor) )
   print('Imu_Y = ' + str(imu_y) )

   t = Twist()

   threshold = 1
   ramp_detection_threshold = 0.00001
   
   
   if abs(imu_y) > ramp_detection_threshold:
      print('Oh no! Ramp detected!')
      print('Moving back')
      for i in range(5):
         t.linear.x = -2
         t.angular.z = 0
         time.sleep(0.5)
      print('Moving circular') 
      for i in range(5):
         t.linear.x = -0.3
         t.angular.z = -2
         time.sleep(0.5)

   elif f_sensor > threshold and lf_sensor > threshold and rf_sensor > threshold :
      print('1. Moving forward')
      t.linear.x = 0.5
      t.angular.z = 0
   elif f_sensor < threshold and lf_sensor > threshold and rf_sensor > threshold:
      print('2. Front Obstacle')
      t.linear.x = -0.3
      # Se temos um obstáculo na frente, decide pra qual lado é melhor fugir
      if lf_sensor <= rf_sensor:
         t.angular.z = -3
      else:
         t.angular.z = 3
   elif f_sensor > threshold and lf_sensor > threshold and rf_sensor < threshold:
      print('3. Right Obstacle')
      t.linear.x = -0.3
      t.angular.z = 1
   elif f_sensor > threshold and lf_sensor < threshold and rf_sensor > threshold:
      print('4. Left Obstacle')
      t.linear.x = -0.3
      t.angular.z = -1
   elif f_sensor < threshold and lf_sensor > threshold and rf_sensor < threshold:
      print('5. Front and Right Obstacles')
      t.linear.x = -0.3
      t.angular.z = 1
   elif f_sensor < threshold and lf_sensor < threshold and rf_sensor > threshold:
      print('6. Front and Left Obstacles')
      t.linear.x = -0.3
      t.angular.z = -1
   elif f_sensor < threshold and lf_sensor < threshold and rf_sensor < threshold:
      print('7. Front, Right and Left Obstacles')
      t.linear.x = -0.3
      t.angular.z = 1
   elif f_sensor > threshold and lf_sensor < threshold and rf_sensor < threshold:
      print('8. Right and Left Obstacles')
      t.linear.x = -0.3
      t.angular.z = 1
   else:
      print('We shouldnt be here. :( ')

   pub.publish(t)


 
if __name__ == '__main__':
   rospy.init_node("obstacle_check_node")
   # Subscreve ao scan para usar os sensores para evitar obstaculos
   rospy.Subscriber("/scan", LaserScan, callback_laser)
   # Subscreve a IMU pra detectar que está na rampa e sair dela. Caso contrario fica preso nela.
   rospy.Subscriber("/imu0", Imu, callback_imu)
   # Publica comandos de movimentação pro robô
   pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
   print("Comando SMB sob obstáculos inicializado com sucesso!")
   rospy.spin() # this will block untill you hit Ctrl+C

