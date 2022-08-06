#!/usr/bin/env python3

import rospy
import sys
import argparse
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray

import math
#import board
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
import time
import adafruit_motor.servo
import RPi.GPIO as GPIO

avoidObjectFlag = False
wideAngleFlag = False
angleFlag = False
rightWithinRange = False

def Sero_Motor_Initialization():
   i2c_bus = busio.I2C(3,2)
   print('2')
   pca = PCA9685(i2c_bus)
   print('3')
   pca.frequency = 100
   return pca

#bit = 65535

def Motor_StartUp(pca):
    print('Starting Motor Start Up Sequence')
    pca.channels[11].duty_cycle = math.floor(.15*65535)
    time.sleep(5)
    pca.channels[11].duty_cycle = math.floor(.2*65535)
    time.sleep(3)
    pca.channels[11].duty_cycle = math.floor(.15*65535)
    time.sleep(3)
    pca.channels[11].duty_cycle = math.floor(.1*65535)
    time.sleep(3)
    pca.channels[11].duty_cycle = math.floor(.15*65535)
    time.sleep(3)
    print('Start Up Complete')


def Motor_Speed(pca, percent, channel = 11):
    pca.channels[channel].duty_cycle = math.floor(percent*65535)
    print(percent)


def Steering(pca, angle):
   if angle > 160:
      angle = 160
   if angle < -90:
      angle = -90

   angle = angle+90
   angle = angle*0.7
   duty = ((angle/180)*6553)+6553


   pca.channels[14].duty_cycle= math.floor(duty)

pca = Sero_Motor_Initialization()
Motor_StartUp(pca)
Motor_Speed(pca, .15, 11)
time.sleep(5)
Motor_Speed(pca, .157, 11)
time.sleep(5)

GPIO.cleanup()
#for i in range(250):
#   Steering(pca, -(i-90))
#   time.sleep(.05)
#print("steering -45")


def callback(data): # Right ultrasonic sensor topic callback
   #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
   print("Right Ultrasonic Sensor: " + str(data.data))
   if data.data < 60.000:
      rightWithinRange == True
      Steering(pca,(-8*data.data) + 480)
      #Steering(pca, 160)
      Motor_Speed(pca, .157, 11) #change to .05
      avoidObjectFlag = True
   else:
      if angleFlag == False:
         Steering(pca, 0)
      if wideAngleFlag == False:
         Motor_Speed(pca, .157, 11)
      if avoidObjectFlag == True:
         time.sleep(3) # Vary based on current speed
      avoidObjectFlag = False
      rightWithinRange == False

def callback2(data):
   #data.data[0] is angle, data.data[1] is left intersection boolean (0.0 is false, 1.0 is true)
   #data.data[2]right intersection boolean (0.0 is false, 1.0 is true)
   rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data[0])
   print("Left Intersection Val: " + str(data.data[1]))
   print("Right Intersection Val: " + str(data.data[2]))

   if avoidObjectFlag == True:
      return

   if data.data[1] == 0 and data.data[2] == 0:
      if data.data[0] > 1:
         Steering(pca, -2.2*data.data[0]) #.5*data.data[0]
         print("Steering angle arg: " + str(-2.2*data.data[0]))
         angleFlag = True
         if data.data[0] > 40:
            wideAngleFlag = True # override ultrasonic control callback
            Motor_Speed(pca, .05, 11)
         else:
            wideAngleFlag = False
      elif data.data[0] < -1:
         Steering(pca, -2.2*data.data[0])
         print("Steering angle arg: " + str(-2.2*data.data[0]))
         angleFlag = True
         if data.data[0] < -40:
            wideAngleFlag = True # override ultrasonic control callback
            Motor_Speed(pca, .05, 11)
         else:
            wideAngleFlag = False
      else:
         Steering(pca, 0)
         #Motor_Speed(pca, .15, 11)
         angleFlag = False
   
#   else:
#      Steering(pca, 0)
#      time.sleep(1.5) # vary
#      if data.data[1] == 1:
#         Steering(pca, 90)
#      if data.data[2] == 1:
#         Steering(pca, -90)
#      time.sleep(1.8) # Vary based on current speed

#      Steering(pca, 0)
               
def callback3(data): # Left ultrasonic sensor topic callback
   #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
   print("Left Ultrasonic Distance: " + str(data.data))
   if data.data < 60.000 and rightWithinRange == False:
      Steering(pca,-((-8*data.data) + 480))
      #Steering(pca, -160)
      Motor_Speed(pca, .157, 11) # Change to .05
      avoidObjectFlag = True
   else:
      if avoidObjectFlag == True:
         time.sleep(3) # Vary based on current speed
      avoidObjectFlag = False
    
      
def listener():
   rospy.init_node('pwm_node', anonymous=True)
   sub = rospy.Subscriber('test_topic_arman', Float32, callback)
   sub2 = rospy.Subscriber('cam_topic_william', Float32MultiArray, callback2)
   sub3 = rospy.Subscriber('ultra_topic_left', Float32, callback3)
   rospy.spin()

if __name__ == '__main__':
   while True:
      try:
         listener()
      except KeyboardInterrupt:
         Motor_Speed(pca, .05, 11)
         time.sleep(5)
         break
