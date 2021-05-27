""" This module extends the module of mpu6050 created by m-rtijn 
the extension allows to control any amount of MPU-6050 devices with Raspberry Pi 
Made by: EIMANSUN
"""
# Standard imports
import time
import math
# External imports
from mpu6050 import mpu6050
import RPi.GPIO as GPIO


class mpu6050_multiple(mpu6050):

    def __init__ (self, AD0_channels, address=0x68, pinmode="BOARD"):
        self.AD0_channels = AD0_channels
        self.address = address
        self.pinmode = pinmode

        # Sets GPIO mode
        if self.pinmode == "BOARD":
            GPIO.setmode(GPIO.BOARD)
        elif self.pinmode == "BCM":
            GPIO.setmode(GPIO.BCM)
        else:
            raise Exception("Invalid pin mode selected! Try BOARD or BCM ")

        # Sets primary adrress for device
        if self.address == 0x68:
            GPIO.setup(self.AD0_channels, GPIO.OUT, initial=GPIO.HIGH)
            self.on_state = GPIO.LOW
            self.off_state = GPIO.HIGH
        elif self.address == 0x69:  
            GPIO.setup(self.AD0_channels, GPIO.OUT, initial=GPIO.LOW)
            self.on_state = GPIO.HIGH
            self.off_state = GPIO.LOW
        else:
            raise Exception("Invalid address of MPU6050 try 0x68 or 0x69")

        # Wakes up all MPU-6050 devices
        for n, ch in enumerate(AD0_channels):
            print(f"Starting MPU6050 No.{n}")
            GPIO.output(ch, self.on_state)
            super().__init__(self.address)
            GPIO.output(ch, self.off_state)


    def get_temp_all(self):
        """Returns the list of temperatures in degrees celcius of each MPU-6050."""
        
        temp_dict = {}
        for n, ch in enumerate(self.AD0_channels):
            GPIO.output(ch, self.on_state)
            temp_dict[n] = self.get_temp()
            GPIO.output(ch, self.off_state)
        return temp_dict


    def set_accel_range_all(self, accel_range):
        """Sets range of each accelometer."""

        for ch in self.AD0_channels:
            GPIO.output(ch, self.on_state)
            self.set_accel_range(accel_range)
            GPIO.output(ch, self.off_state)


    def read_accel_range_all(self, raw=False):
        """Reads the range of each accelometer."""

        accel_range_list = []
        for ch in self.AD0_channels:
            GPIO.output(ch, self.on_state)
            accel_range_list.append(self.read_accel_range(raw))
            GPIO.output(ch, self.off_state)
        return accel_range_list


    def get_accel_data_all(self, g=False):
        """Gets and returns the
         X, Y and Z values from all accelerometers.

        If g is True, it will return the data in g
        If g is False, it will return the data in m/s^2"""

        accel_data_dict = {}
        for n, ch in enumerate(self.AD0_channels):
            GPIO.output(ch, self.on_state)
            accel_data_dict[n] = self.get_accel_data(g)
            GPIO.output(ch, self.off_state)
        return accel_data_dict


    def set_gyro_range_all(self, gyro_range):
        """Sets range of each gyro."""

        for ch in self.AD0_channels:
            GPIO.output(ch, self.on_state)
            self.set_gyro_range(gyro_range)
            GPIO.output(ch, self.off_state)


    def read_gyro_range_all(self, raw=False):
        """Reads the range of each gyro."""

        gyro_range_list = []
        for ch in self.AD0_channels:
            GPIO.output(ch, self.on_state)
            gyro_range_list.append(self.read_gyro_range(raw))
            GPIO.output(ch, self.off_state)
        return gyro_range_list


    def get_gyro_data_all(self):
        """Gets and returns the
         X, Y and Z values from all gyros.

        If g is True, it will return the data in g
        If g is False, it will return the data in m/s^2"""

        gyro_data_dict = {}
        for n, ch in enumerate(self.AD0_channels):
            GPIO.output(ch, self.on_state)
            gyro_data_dict[n] = self.get_gyro_data()
            GPIO.output(ch, self.off_state)
        return gyro_data_dict


    def get_all_sensor_data(self, g=False):
        """Reads and returns all the available data from each sensor"""
        
        all_data_dict = {}
        for n, ch in enumerate(self.AD0_channels):
            GPIO.output(ch, self.on_state)
            temp = self.get_temp()
            accel = self.get_accel_data(g)
            gyro = self.get_gyro_data()
            all_data_dict[n] = {"Temperature":temp, "Accelerometer":accel, "Gyroscope":gyro}
            GPIO.output(ch, self.off_state)
        return all_data_dict


    def get_accel_angle_data(self, radians=False):
        """Transforms accelometer data to x and y angles
        in radians or degrees"""

        angle_dict = {}
        for n, ch in enumerate(self.AD0_channels):
            GPIO.output(ch, self.on_state)
            accel_data = self.get_accel_data(g=True)

            if radians == False:
                yAngle = math.degrees(math.atan2(accel_data['z'], accel_data['x']))
                xAngle = math.degrees(math.atan2(accel_data['z'], accel_data['y']))
            else:
                yAngle = math.atan2(accel_data['z'], accel_data['x'])
                xAngle = math.atan2(accel_data['z'], accel_data['y'])
           
            angle_dict[n] = {"x":xAngle, "y":yAngle}
            GPIO.output(ch, self.off_state)
        return angle_dict
 

if __name__ == "__main__":
    """Assuming that the address is 0x68
    and that two MPU-6050 are connected to pin 11 and 13 of RPi
    prints out all sensros data as a demo""" 
    
    channels = (11, 13)
    mult_mpu = mpu6050_multiple(channels)
    print(mult_mpu.get_all_sensor_data())
