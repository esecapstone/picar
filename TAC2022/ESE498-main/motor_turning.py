#!/usr/bin/env python3
import rospy
import message_filters
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
import math
from board import SCL, SDA
import busio
from Adafruit_PCA9685 import PCA9685 as PCA9685_steer
from adafruit_pca9685 import PCA9685 as PCA9685_motor
import time
from simple_pid import PID
import simple_pid
from csv import reader, writer
import csv

pca_steer = PCA9685_steer()
pca_steer.set_pwm_freq(100)

i2c = busio.I2C(SCL, SDA)
pca_motor = PCA9685_motor(i2c)
pca_motor.frequency = 100

us_data_glob = 999
encoder_data_glob = 0
acc_data_glob = []
setpoint_glob = 1
lidar_data_glob = [999]

motor_channel = 6
steering_channel = 5

next_turn_time = time.time()
ready_to_turn = 1
initial_time = time.time()

pid = PID(1,0.1,0.05,setpoint=setpoint_glob)
pid.sample_time = 0.005
pid.output_limits = (0, 35)

def Motor_Speed(pca_motor, percent, channel = 6):
    pca_motor.channels[channel].duty_cycle = math.floor(percent*65535)
    #print(percent)

def Steering(pca_steer, angle, channel = 5):
    #Converts to 16-bit duty between 10% and 20%
    #duty = ((angle/180)*6553)+6553
    duty = math.floor(((angle/180)*270)+540)
    #duty = math.floor(((angle/180)*270)+90)
    #pca_steer.channels[channel].duty_cycle = math.floor(duty)
    pca_steer.set_pwm(channel, 0, angle)

def encoder_callback(encoder_data):
    global encoder_data_glob
    #rospy.loginfo("Encoder Measurements: %f", encoder_data.data)
    encoder_data_glob = encoder_data.data
    work()

#def ultrasonic_callback(us_data):
    #global us_data_glob
    #rospy.loginfo("Ultrasonic Measurements: %f", us_data.data)
    #us_data_glob = us_data.data
    #work()

#def lt_callback(lt_data):
   #rospy.loginfo("Line Tracker Measurements: %f", lt_data.data)

def lidar_callback(lidar_data):
    global lidar_data_glob
    lidar_data_glob = lidar_data.ranges
    #print("hi")
    rospy.loginfo(min(lidar_data.ranges))
  #rospy.loginfo("Minimum angle: %f", lidar_data.angle_min)
  #rospy.loginfo("Maximum angle: %f", lidar_data.angle_max)
  #rospy.loginfo("Angle increment: %f", lidar_data.angle_increment)
  #print(lidar_data.angle_min)
    #rospy.loginfo("Lidar data: %f", lidar_data.ranges[0])
    #rospy.loginfo("Lidar min range: %f", lidar_data.range_min)
  #print(lidar_data.ranges)

def accelerometer_callback(acc_data):
    global acc_data_glob
    #rospy.loginfo("Accelerometer Data: %s", acc_data.data)
    acc_data_glob.append(acc_data.data)
    work()

#def callback(camera):
   #rospy.loginfo()

def turn():
    global acc_data_glob
    #Motor_Speed(pca_motor, 0.158, motor_channel)
    #Steering(pca_steer, 810, steering_channel)
    #time.sleep(2.25)
    #Steering(pca_steer, 595, steering_channel)
    #print(acc_data_glob)
   # write_to_csv(acc_data_glob,'acc_data.csv')
    #Motor_Speed(pca_motor, 0.150, motor_channel)

def write_to_csv(data, file_name):
    with open(file_name, 'w', newline='') as csvFile:
        #csvFile.truncate()
        for row in data:
            csv_writer = csv.writer(csvFile, delimiter=',', quoting=csv.QUOTE_MINIMAL)
            csv_writer.writerow(row)

def work():
    global ready_to_turn
    global next_turn_time
    global lidar_data_glob
    #filtered_lidar_data = [v for v in lidar_data_glob if not (math.isinf(v) or math.isnan(v))]
    #print(min(filtered_lidar_data))
    #print(lidar_data_glob)
   # Motor_Speed(pca_motor,0.158,motor_channel)
    #if us_data_glob < 1:
    #    Motor_Speed(pca, 0.15, motor_channel)
    #if time.time() < initial_time + 1:
    #    Motor_Speed(pca_motor, 0.158, motor_channel)
    #else:
    #    control_velocity = pid(encoder_data_glob)
    #    motor_percent = 0.15 + (0.05/15.6464)*control_velocity
    #    Motor_Speed(pca_motor,motor_percent,motor_channel)
    #    rospy.loginfo("Encoder Velocity: %s, Control Velocity: %s, Motor Percent: %s", encoder_data_glob, control_velocity, motor_percent)
    if ready_to_turn and time.time() > next_turn_time:
        turn()
        next_turn_time = time.time() + 5

if __name__ == '__main__':
    #Servo_Initialize()
    #print('hi')
    rospy.init_node('motor_turning', anonymous=True)
    rospy.Subscriber('encoder_topic', Float32, encoder_callback)
    rospy.Subscriber('accelerometer_topic', Float32MultiArray, accelerometer_callback)
    #rospy.Subscriber('ultrasonic_topic', Float32, ultrasonic_callback)
    #rospy.Subscriber('linetracker_topic', Float32, lt_callback)
    #rospy.Subscriber('scan', LaserScan, lidar_callback)
    #rospy.Subscriber('lidar_topic', Float32MultiArray, lidar_callback)
    #rospy.Subscriber('camera_topic', fasdfas, camera_callback)
    #while True:
    #    work()
    try:
        rospy.loginfo("Started subscriber node...")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down subscriber!")
        Motor_Speed(pca_motor,0.15,motor_channel)
        lidar.stop()
        time.sleep(1)
        sp.shutdown()