from curses import KEY_PPAGE
from tracemalloc import start
from xxlimited import Null
import iqmotion as iq
import time
import math

import numpy as np
import matplotlib.pyplot as plt
import turtle

# Setup com ports (Subject to change based on where USB is plugged in)
com1 = iq.SerialCommunicator("/dev/cu.usbserial-14110")
com2 = iq.SerialCommunicator("/dev/cu.usbserial-14120")
com3 = iq.SerialCommunicator("/dev/cu.usbserial-14130")
com4 = iq.SerialCommunicator("/dev/cu.usbserial-14140")

# Initialize motors as IQ objects
vertiq1 = iq.Vertiq2306(com1, 0, firmware="servo")
vertiq2 = iq.Vertiq2306(com2, 0, firmware="servo")
vertiq3 = iq.Vertiq2306(com3, 0, firmware="servo")
vertiq4 = iq.Vertiq2306(com4, 0, firmware="servo")
vertiqs = [vertiq1, vertiq2, vertiq3, vertiq4]

# Target Speed (rad/s)
targetSpeed1 = 234.34
targetSpeed2 = 336.36

# Angle offset
motorOff1 = 0 # -.11767334
motorOff2 = 0 #-4.79
motorOff3 = 0
motorOff4 = 0 # .16890137

# P I D
P = .7
I = .4
D = .08
# 1,.5,.1

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
    
    def compute(self, position, delay):
        self.currentTime = time.time()
        self.error = ((self.currentTime-self.startTime) * self.target) + delay - position + self.motorOffset
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
    
    #def findOffset(self, r, theta):
        

class Motor(object):
    def __init__(self, vertiq, KP, KI, KD, target, motorOff):
        global Motor
        self.velocity = 0
        self.vertiq = vertiq
        self.PID = PID(KP, KI, KD, target, motorOff)

    def set_velocity(self, velocity):
        self.velocity = velocity
    
    def get_velocity(self):
        return self.velocity

def graph(time1,angle1):
    plt.plot(time1, angle1, label="Virtual")
    plt.show()

motor1 = Motor(vertiq1, P, I, D, targetSpeed1, motorOff1)
motor2 = Motor(vertiq2, P, I, D, targetSpeed1, motorOff2)
motor3 = Motor(vertiq3, P, I, D, targetSpeed2, motorOff3)
motor4 = Motor(vertiq4, P, I, D, targetSpeed2, motorOff4)
motors = [motor1, motor2, motor3, motor4]

# Set initial speed of motors

for motor in motors:
    motor.vertiq.set("multi_turn_angle_control", "trajectory_angular_displacement", 0)
    motor.vertiq.set("multi_turn_angle_control", "trajectory_duration", 1)

"""
motor1.vertiq.set("multi_turn_angle_control", "trajectory_angular_displacement", math.pi/2)
motor1.vertiq.set("multi_turn_angle_control", "trajectory_duration", 1)
motor2.vertiq.set("multi_turn_angle_control", "trajectory_angular_displacement", 0)
motor2.vertiq.set("multi_turn_angle_control", "trajectory_duration", 1)
motor3.vertiq.set("multi_turn_angle_control", "trajectory_angular_displacement", math.pi/2)
motor3.vertiq.set("multi_turn_angle_control", "trajectory_duration", 1)
motor4.vertiq.set("multi_turn_angle_control", "trajectory_angular_displacement", 0)
motor4.vertiq.set("multi_turn_angle_control", "trajectory_duration", 1)
"""

time.sleep(1.5)

# Store start time of Program
startTime = time.time()
print("100")
targetSpeed = 100
for motor in motors:
    motor.vertiq.set("multi_turn_angle_control", "ctrl_velocity", targetSpeed)
    motor.PID.startTime = startTime
    motor.PID.prevTime = startTime

time.sleep(3)

# Store start time of Program
startTime = time.time()
print("175")
targetSpeed = 175
for motor in motors:
    motor.vertiq.set("multi_turn_angle_control", "ctrl_velocity", targetSpeed)
    motor.PID.startTime = startTime
    motor.PID.prevTime = startTime

time.sleep(3)

# Store start time of Program
startTime = time.time()
print("250")
targetSpeed = 225
for motor in motors:
    motor.vertiq.set("multi_turn_angle_control", "ctrl_velocity", targetSpeed)
    motor.PID.startTime = startTime
    motor.PID.prevTime = startTime

time.sleep(3)

# Store start time of Program
startTime = time.time()
print("300")
targetSpeed = 275
for motor in motors:
    motor.vertiq.set("multi_turn_angle_control", "ctrl_velocity", targetSpeed)
    motor.PID.startTime = startTime
    motor.PID.prevTime = startTime

time.sleep(3)

# Store start time of Program
startTime = time.time()
print("350")
targetSpeed = 325
for motor in motors:
    motor.vertiq.set("multi_turn_angle_control", "ctrl_velocity", targetSpeed)
    motor.PID.startTime = startTime
    motor.PID.prevTime = startTime

time.sleep(3)

# Store start time of Program
startTime = time.time()
print("375")
targetSpeed = 336
for motor in motors:
    motor.vertiq.set("multi_turn_angle_control", "ctrl_velocity", targetSpeed)
    motor.PID.startTime = startTime
    motor.PID.prevTime = startTime

time.sleep(3)

finalDelay = 3*100 + 3*175 + 3*250 + 3*300 + 3*350 + 3*375 + 3*400 + 3*425 + 3*450 + 3*465

# Store start time of Program
startTime = time.time()

targetSpeed = 465.69
for motor in motors:
    motor.vertiq.set("multi_turn_angle_control", "ctrl_velocity", targetSpeed)
    motor.PID.startTime = startTime
    motor.PID.prevTime = startTime

    
while True:
    for index, motor in enumerate(motors):
        obsDisplacement = motor.vertiq.get("multi_turn_angle_control", "obs_angular_displacement")
        if obsDisplacement is not None:
            velocity = motor.PID.compute(obsDisplacement, finalDelay)
            motor.vertiq.set("multi_turn_angle_control", "ctrl_velocity", targetSpeed + velocity)

            #print("Motor " + str(index) + ": " + str(obsDisplacement))
            # print("Motor %s: Position Error: %s" % (index+1, motor.PID.error))
            #if (index == 1):
                #print("Motor %s: Obersved Position: %s " % (index+1, obsDisplacement))

            
            #print(motor1.PID.error)