#!/usr/bin/env python3
import RPi.GPIO as IO
import time
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
import sys
import argparse
import busio
import smbus
from time import sleep
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
import math

IO.setwarnings(False)
IO.setmode(IO.BCM)

pin_num = 26
IO.setup(pin_num,IO.IN,IO.PUD_UP)

def read():
    gear_rotations = 0
    time_start = time.time()
    time_end = time.time()
    ready_to_go = False
    while gear_rotations < 5:
        curr_pin_val = IO.input(pin_num)
        if curr_pin_val == 1 and ready_to_go == False:
            ready_to_go = True
            gear_rotations += 1
        if curr_pin_val == 0 and ready_to_go == True:
            ready_to_go = False
        if time.time() > time_start + 0.5:
            return 0
        #print(gear_rotations)
    #print('hi!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
    time_end = time.time()
    #speed = (math.pi*.111 / (time_end - time_start)) / 1000
    speed = (2*math.pi*.111) / (time_end - time_start)
    time_start = time.time()
    return speed

def talker():
    pub = rospy.Publisher('encoder_topic', Float32, queue_size=1)
    rospy.init_node('encoder_talk',anonymous = True)
    rate = rospy.Rate(10)
    data = read()
    pub.publish(data)
    rate.sleep()

if __name__ == '__main__':
   while True:
      try:
         talker()
      except KeyboardInterrupt:
         break

GPIO.cleanup(0)
