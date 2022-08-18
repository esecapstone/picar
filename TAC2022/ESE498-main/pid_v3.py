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
import board
import adafruit_mpu6050
import numpy as np

pca_steer = PCA9685_steer()
pca_steer.set_pwm_freq(100)

i2c = busio.I2C(SCL, SDA)
pca_motor = PCA9685_motor(i2c)
pca_motor.frequency = 100

desired_angle = 595
turn_distance = 2.2
us_data_glob = [999, 999, 999, 999, 999]
latest_us_val = 999
encoder_data_glob = 0
lidar_data_glob = [999]
filtered_lidar_readings = [999, 999, 999, 999, 999]
encoder_speeds = []
acc_data_glob = []
speed_setpoint_glob = 3 #m/s
angle_setpoint_glob = 0
current_angle = 0
turning = 0

motor_channel = 6
steering_channel = 5

next_turn_time = time.time() + 3
finish_turn_time = time.time() + 4
done_turning = False
ready_to_record = 1
next_recording_time = time.time() + 120

#pid = PID(1,0.1,0.05,setpoint=speed_setpoint_glob)
pid = PID(3,0,0,setpoint=speed_setpoint_glob)
pid.sample_time = 0.01

angle_pid = PID(0.00000001,0,0.001,setpoint=angle_setpoint_glob)
angle_pid.sample_time = 0.01
angle_pid.output_limits = (-10,10)

def Motor_Speed(pca_motor, percent, channel = 6):
    pca_motor.channels[channel].duty_cycle = math.floor(percent*65535)

def Steering(pca_steer, angle, channel = 5):
    pca_steer.set_pwm(channel, 0, angle)

def encoder_callback(encoder_data):
    global encoder_data_glob
    global encoder_speeds
    encoder_data_glob = encoder_data.data
    encoder_speeds.append([encoder_data.data])
    work()

#def ultrasonic_callback(us_data):
    #global us_data_glob
    #us_data_glob.append(us_data.data)
    #latest_us_val = us_data.data
    #work()

def accelerometer_callback(acc_data):
    global acc_data_glob
    global current_angle
    acc_data_glob = acc_data.data
    if time.time() > 1:
        current_angle += float(acc_data_glob[5])*180/math.pi*0.1
    work()

def lidar_callback(lidar_data):
    global lidar_data_glob
    if time.time() > 1:
        lidar_data_glob = lidar_data.ranges
    #rospy.loginfo(min(lidar_data.ranges))
    work()

def turn():
    global turning
    global acc_data_glob
    turning = True
    finish_turn = time.time() + 1
    Steering(pca_steer, 700, steering_channel)
    if time.time() > finish_turn:
        Steering(pca_steer, 595, steering_channel)
        turning = False

def write_to_csv(data, file_name):
    with open(file_name, 'w', newline='') as csvFile:
        for row in data:
            csv_writer = csv.writer(csvFile, delimiter=',', quoting=csv.QUOTE_MINIMAL)
            csv_writer.writerow(row)

# y = 628.88x - 97.192
def work():
    global ready_to_turn
    global next_turn_time
    global ready_to_record
    global acc_data_glob
    global encoder_speeds
    global us_data_glob
    global current_angle
    global angle_setpoint_glob
    global finish_turn_time
    global done_turning
    global desired_angle
    global lidar_data_glob
    global turn_distance
    global filtered_lidar_readings

    #filtered_lidar_data = [v for v in lidar_data_glob if not (math.isinf(v) or math.isnan(v))]
    #filtered_lidar_readings.append(np.mean(filtered_lidar_data))
    filtered_lidar_readings.append(lidar_data_glob[0])
    avg_distance = np.sum(filtered_lidar_readings[-5:]) / 5
    #print(avg_distance)
    Steering(pca_steer,595,steering_channel)
    #if time.time() > 0:
    if time.time() > next_recording_time:
        Motor_Speed(pca_motor,0.150,motor_channel)
    else:
        if latest_us_val < 0 :
            print("Object detected")
            Motor_Speed(pca_motor,0.150,motor_channel)
        else:
            correction_angle = pid(current_angle)
            pid_steering(correction_angle)

            control_velocity = pid(encoder_data_glob)
            pid_velocity(control_velocity)
            if avg_distance < turn_distance and time.time() > next_turn_time:
                next_turn_time = time.time() + 5
                desired_angle = 700
                finish_turn_time = time.time() + 1

                done_turning = True
            elif done_turning and time.time() > finish_turn_time:
                done_turning = False
                desired_angle = 595
                current_angle -= 90
            elif not done_turning:
                pid_steering(correction_angle)
            else:
                Steering(pca_steer,desired_angle,steering_channel)
    #if time.time() > next_recording_time and ready_to_record:
    #    ready_to_record = 0
    #   # write_to_csv(acc_data_glob,'acc_data.csv')
    #    write_to_csv(encoder_speeds,'encoder_data.csv')
    #    print("WROTE TO FILE ---------------------------------------------------------")

def pid_velocity(control_velocity):
    if control_velocity < 2.17:
        Motor_Speed(pca_motor,0.158,motor_channel)
        #print("Going .158 or 2.17m/s", encoder_data_glob)
    elif control_velocity > 5.45:
        Motor_Speed(pca_motor,0.164,motor_channel)
        #print("Going .164 or 5.94 m/s", encoder_data_glob)
    else:
        motor_percent = (control_velocity + 97.192) / 625.88
        #print("Inbetween: ",motor_percent)
        Motor_Speed(pca_motor,motor_percent,motor_channel)

def pid_steering(correction_angle):
    max_angle = 40
    if correction_angle < -max_angle:
        Steering(pca_steer,int(595-max_angle),steering_channel)
    elif correction_angle > max_angle:
        Steering(pca_steer,int(595+max_angle),steering_channel)
    else:
        Steering(pca_steer,int(595+correction_angle),steering_channel)

if __name__ == '__main__':
    rospy.init_node('motor_turning', anonymous=True)
    rospy.Subscriber('encoder_topic', Float32, encoder_callback)
    rospy.Subscriber('accelerometer_topic', Float32MultiArray, accelerometer_callback)
    #rospy.Subscriber('ultrasonic_topic', Float32, ultrasonic_callback)
    rospy.Subscriber('scan', LaserScan, lidar_callback)
    try:
      rospy.loginfo("Started subscriber node...")
      rospy.spin()
    except rospy.ROSInterruptException:
      rospy.loginfo("Shutting down subscriber!")
      sp.shutdown()
