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
import board
import adafruit_mpu6050

i2c = board.I2C()
mpu = adafruit_mpu6050.MPU6050(i2c)

def read():

    acc = mpu.acceleration
    gyro = mpu.gyro
    measurements = [float(acc[0]), float(acc[1]), float(acc[2]), float(gyro[0]), float(gyro[1]), float(gyro[2])]

    return Float32MultiArray(data=measurements)


def talker():
    pub = rospy.Publisher('accelerometer_topic', Float32MultiArray, queue_size=1)
    rospy.init_node('accelerometer_talk',anonymous = True)
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
