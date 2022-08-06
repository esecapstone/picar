#!/usr/bin/env python3
#import Rpi.GPIO as GPIO
import time
import rospy
from std_msgs.msg import Float32
#import Rpi.GPIO as GPIO
import sys
import argparse
#import busio
import smbus
import RPi.GPIO as GPIO
from time import sleep

#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)
 
#set GPIO Pins
GPIO_TRIGGER = 18
GPIO_ECHO = 24
 
#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

def test():
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)
 
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
 
    StartTime = time.time()
    StopTime = time.time()
 
    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()
    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time() 
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2
 
    return distance
 

def talker():
   pub = rospy.Publisher('test_topic_arman', Float32, queue_size=1)
   rospy.init_node('test_node', anonymous = True)
   rate = rospy.Rate(12)
   data = test()
   pub.publish(data)
   rate.sleep()

if __name__ == '__main__':
   while True:
      try:
         talker()
      except KeyboardInterrupt:
         break
