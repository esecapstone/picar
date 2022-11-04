import math
from board import SCL,SDA
import busio
from adafruit_pca9685 import PCA9685
import time
import adafruit_motor.servo

def Servo_Motor_Initialization():
   i2c_bus = busio.I2C(SCL,SDA)
   pca = PCA9685(i2c_bus)
   pca.frequency = 100
   return pca

def Motor_Start(pca):
   x = input("Press and hold ez button. When RED LED turns red wait 3 seconds.")
   Motor_Speed(pca, 1)
   time.sleep(3)
   Motor_Speed(pca, 0)
   Motor_Speed(pca, -1)
   time.sleep(3)
   Motor_Speed(pca, 0)
   time.sleep(3)

def Motor_Speed(pca,percent):
   #converts a -1 to 1 value to 16-bit duty cycle
   speed = ((percent) * 3276) + 65535 * 0.15
   pca.channels[6].duty_cycle = math.floor(speed)
   print(speed/65535)

#initialization
pca = Servo_Motor_Initialization()
Motor_Start(pca)

Motor_Speed(pca, 0.2)
time.sleep(0.5)
Motor_Speed(pca, 0.16)
time.sleep(3)
Motor_Speed(pca, 0)
Footer
