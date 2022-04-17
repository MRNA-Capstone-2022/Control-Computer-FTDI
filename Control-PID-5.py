from __future__ import print_function
from curses import KEY_PPAGE
from tracemalloc import start
from xxlimited import Null
import iqmotion as iq
import time
import math
import threading

import numpy as np
import matplotlib.pyplot as plt
import turtle

# Setup com ports (Subject to change based on where USB is plugged in)
com1 = iq.SerialCommunicator("/dev/cu.usbserial-14510")
com2 = iq.SerialCommunicator("/dev/cu.usbserial-14520")
com3 = iq.SerialCommunicator("/dev/cu.usbserial-14530")
com4 = iq.SerialCommunicator("/dev/cu.usbserial-14540")

# Initialize motors as IQ objects
vertiq1 = iq.Vertiq2306(com1, 0, firmware="servo")
vertiq2 = iq.Vertiq2306(com2, 0, firmware="servo")
vertiq3 = iq.Vertiq2306(com3, 0, firmware="servo")
vertiq4 = iq.Vertiq2306(com4, 0, firmware="servo")
vertiqs = [vertiq1, vertiq2, vertiq3, vertiq4]

# Target Speed (rad/s)
targetSpeed = 5

# Angle offset
motorOff1 = 0#math.pi/2
motorOff2 = 0 #-4.79
motorOff3 = 0#math.pi/2
motorOff4 = 0

# P I D
P = .03
I = .0005
D = .01
# 1,1,.1

class PID(object):
    def __init__(self, KP, KI, KD, target, motorOff, startTime):
        self.kp = KP
        self.ki = KI
        self.kd = KD
        self.target = target
        self.error = 0
        self.error_integral = 0
        self.error_last = 0
        self.error_derivative = 0
        self.output = 0
        self.prevTime = 0
        self.currentTime = startTime
        self.motorOffset = motorOff
        self.displacement = 0

    def compute(self, position):
        self.error += self.updateVM(time.time()) - position + self.motorOffset
        self.error_integral += self.error * (self.currentTime - self.prevTime)
        self.error_derivative = (self.error - self.error_last) / (self.currentTime - self.prevTime)
        self.error_last = self.error
        self.output = self.kp*self.error + self.ki*self.error_integral + self.kd*self.error_derivative

        print(self.error)

        self.prevPosition = position

        # Limits
        if self.output > 50:
            self.output = 50
        if self.output < -50:
            self.output = -50
        return self.output
    
    def updateVM(self, time):
        self.prevTime = self.currentTime
        self.currentTime = time
        newDisplacement = (self.currentTime - self.prevTime) * self.target
        self.displacement += newDisplacement
        return self.displacement
        

class Motor(object):
    def __init__(self, vertiq, KP, KI, KD, target, motorOff, startTime):
        global Motor
        self.velocity = 0
        self.vertiq = vertiq
        self.PID = PID(KP, KI, KD, target, motorOff, startTime)
        self.active = True

class VirtualMotor(object):
    def __init__(self, startTime):
        global VirtualMotor
        self.prevTime = startTime
        self.velocity = 0
        self.displacement = 0
        self.active = True
    
    def compute(self):
        currentTime = time.time()
        self.displacement += (currentTime - self.prevTime) * self.velocity
        self.prevTime = currentTime
    
    def set_velocity(self, velocity):
        self.velocity = velocity

    def getDisplacement(self):
        return self.displacement


def motorThread(motor):
    while motor.active:
        obsDisplacement = motor.vertiq.get("multi_turn_angle_control", "obs_angular_displacement")
        if obsDisplacement is not None:
            velocity = motor.PID.compute(obsDisplacement)
            motor.vertiq.set("multi_turn_angle_control", "ctrl_velocity", targetSpeed + velocity)

startTime = time.time()

motor1 = Motor(vertiq1, P, I, D, targetSpeed, motorOff1, startTime)
motor2 = Motor(vertiq2, P, I, D, targetSpeed, motorOff2, startTime)
motor3 = Motor(vertiq3, P, I, D, targetSpeed, motorOff3, startTime)
motor4 = Motor(vertiq4, P, I, D, targetSpeed, motorOff4, startTime)
motors = [motor1, motor2, motor3, motor4]


# Set initial speed of motors
for motor in motors:
    motor.vertiq.set("multi_turn_angle_control", "trajectory_angular_displacement", 0)
    motor.vertiq.set("multi_turn_angle_control", "trajectory_duration", 1)

time.sleep(1.5)

# Store start time of Program
startTime = time.time()

for motor in motors:
    motor.vertiq.set("multi_turn_angle_control", "ctrl_velocity", targetSpeed)

try:
    t1=(threading.Thread(target=motorThread, args=(motor1, ))).start()
    #t2=(threading.Thread(target=motorThread, args=(motor2, ))).start()
    #t3=(threading.Thread(target=motorThread, args=(motor3, ))).start()
    #t4=(threading.Thread(target=motorThread, args=(motor4, ))).start()

except:
    print("Error: unable to start thread")

while True:
    targetSpeed = float(input("Enter Target Speed: "))
    