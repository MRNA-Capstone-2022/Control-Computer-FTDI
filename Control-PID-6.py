from curses import KEY_PPAGE
from curses.ascii import isdigit
import threading
from tracemalloc import start
from xxlimited import Null
import iqmotion as iq
import time
import math
from matplotlib import type1font

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
targetSpeed = 0

def leftPairOffset(angle, d):
    oppA = d * math.sin(angle)
    j = abs(.53 - oppA)
    h = math.sqrt(d*d - oppA)
    finalAngle = math.atan(h/j)
    print(finalAngle * math.pi / 180)
    return finalAngle * math.pi / 180


# Phase Angle
theta = math.pi # (rad)
r = 0 # (m)

# Motor 1 and 2 (Motor 2 always relative to Motor 1)
# Motor 2 will have offset 0 and Motor 1 will adjust
#oppA = r * Math.sin()

# Angle offset
motorOff1 = math.pi/2
motorOff2 = 0 #-4.79
motorOff3 = math.pi/2
motorOff4 = 0

# P I D
P = .8
I = .28
D = .08
# 1,1,.1
# No PID
#P = 0
#I = 0
#D = 0

class PID(object):
    def __init__(self, KP, KI, KD, target, motorOff):
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
        self.currentTime = 0
        self.motorOffset = motorOff
        self.startTime = 0
        self.displacement = 0
    
    def compute(self, position):
        self.updateDisplacement(time.time()) 
        self.error = self.displacement - position + self.motorOffset
        self.error_integral += self.error * (self.currentTime - self.prevTime)
        self.error_derivative = (self.error - self.error_last) / (self.currentTime - self.prevTime)
        self.error_last = self.error
        self.output = self.kp*self.error + self.ki*self.error_integral + self.kd*self.error_derivative

        self.prevTime = self.currentTime
        
        # Limits
        if self.output > 50:
            self.output = 50
        if self.output < -50:
            self.output = -50
        return self.output
    
    def updateDisplacement(self, time):
        self.currentTime = time
        newDisplacement = (self.currentTime - self.prevTime) * self.target
        self.displacement += newDisplacement

class Motor(object):
    def __init__(self, vertiq, KP, KI, KD, target, motorOff):
        global Motor
        self.velocity = 0
        self.vertiq = vertiq
        self.PID = PID(KP, KI, KD, target, motorOff)
        self.active = False

    def set_velocity(self, velocity):
        self.velocity = velocity
    
    def get_velocity(self):
        return self.velocity

def motorThread(motor):
    while motor.active:
        #print("Motor go")
        obsDisplacement = motor.vertiq.get("multi_turn_angle_control", "obs_angular_displacement")
        if obsDisplacement is not None:
            velocity = motor.PID.compute(obsDisplacement)
            if velocity is not None:
                motor.vertiq.set("multi_turn_angle_control", "ctrl_velocity", motor.PID.target + velocity)
    return

def graph(x,y,z):
    plt.plot(x, y, label="Virtual")
    plt.plot(x, z, label="Observed")
    plt.show()

motor1 = Motor(vertiq1, P, I, D, targetSpeed, motorOff1)
motor2 = Motor(vertiq2, P, I, D, targetSpeed, motorOff2)
motor3 = Motor(vertiq3, P, I, D, targetSpeed, motorOff3)
motor4 = Motor(vertiq4, P, I, D, targetSpeed, motorOff4)
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
    motor.PID.currentTime = startTime
    motor.PID.prevTime = startTime

globalDisplacement = 0
prevUpdateTime = startTime
prevTargetSpeed = targetSpeed

while True:
    for motor in motors:
        motor.active = True

    t1=threading.Thread(target=motorThread, args=(motor1, ))
    t2=threading.Thread(target=motorThread, args=(motor2, ))
    t3=threading.Thread(target=motorThread, args=(motor3, ))
    t4=threading.Thread(target=motorThread, args=(motor4, ))

    t1.start()
    t2.start()
    t3.start()
    t4.start()

    inputType = input("Enter 's' to change speed: ")

    if not inputType.strip().isdigit():
        if (inputType == 's'):
            targetSpeed = float(input("Enter Target Speed: "))

            updateTime = time.time()
            globalDisplacement += (updateTime - prevUpdateTime) * prevTargetSpeed

            for motor in motors:
                motor.active = False
                motor.PID.target = targetSpeed

                motor.PID.currentTime = updateTime
                motor.PID.prevTime = updateTime

                motor.PID.displacement = globalDisplacement

            prevUpdateTime = updateTime
            prevTargetSpeed = targetSpeed

            t1.join()
            t2.join()
            t3.join()
            t4.join()
    