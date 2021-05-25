#! /usr/bin/python3

from mpu6050 import mpu6050
import RPi.GPIO as GPIO
import time
import math

# Refresh frequency in hz
loopTime = 100 

GPIO.setmode(GPIO.BOARD)
# Address 'AD0' pins for any number of MPU6050
AD = (11, 13)
GPIO.setup(AD, GPIO.OUT, initial=GPIO.HIGH)

# Wake all sensros up
for n, ch in enumerate(AD):
    print(f"Starting MPU6050 No.{n}")
    GPIO.output(ch, GPIO.LOW)
    sensor = mpu6050(0x68)
    sensor.set_accel_range(sensor.ACCEL_RANGE_2G)
    GPIO.output(ch, GPIO.HIGH)
    time.sleep(1)

def getAngle(accelData):
    yAngle = math.degrees(math.atan2(accelData['z'], accelData['x']))
    xAngle = math.degrees(math.atan2(accelData['z'], accelData['y']))
    return (xAngle, yAngle)
 
try:
    while True:
        accelDataList = []
        # Set address pin to low and collect data from active sensor
        for ch in AD:
            GPIO.output(ch, GPIO.LOW)
            accelDataList.append(getAngle(sensor.get_accel_data()))
            GPIO.output(ch, GPIO.HIGH)

        print(accelDataList)
        time.sleep(1/loopTime)
except KeyboardInterrupt: # If CTRL+C is pressed, exit cleanly:
    print("Keyboard interupt")
finally:
    print("Clean up")
    GPIO.cleanup() # Clean up all GPIO

