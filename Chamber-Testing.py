from curses import KEY_PPAGE
from curses.ascii import isdigit
import enum
import threading
from tracemalloc import start
from xxlimited import Null
import iqmotion as iq
import time
import math
import random
from matplotlib import type1font

import numpy as np
import matplotlib.pyplot as plt
import turtle

# Setup com ports (Subject to change based on where USB is plugged in)
com1 = iq.SerialCommunicator("/dev/cu.usbserial-14410")
com2 = iq.SerialCommunicator("/dev/cu.usbserial-14420")
com3 = iq.SerialCommunicator("/dev/cu.usbserial-14430")
com4 = iq.SerialCommunicator("/dev/cu.usbserial-14440")

# Initialize motors as IQ objects
vertiq1 = iq.Vertiq2306(com1, 0, firmware="servo")
vertiq2 = iq.Vertiq2306(com2, 0, firmware="servo")
vertiq3 = iq.Vertiq2306(com3, 0, firmware="servo")
vertiq4 = iq.Vertiq2306(com4, 0, firmware="servo")
vertiqs = [vertiq1, vertiq2, vertiq3, vertiq4]


vertiq1.set("multi_turn_angle_control", "trajectory_angular_displacement", 0)
vertiq1.set("multi_turn_angle_control", "trajectory_duration", 1)
vertiq2.set("multi_turn_angle_control", "trajectory_angular_displacement", math.pi/2)
vertiq2.set("multi_turn_angle_control", "trajectory_duration", 1)
vertiq3.set("multi_turn_angle_control", "trajectory_angular_displacement", math.pi)
vertiq3.set("multi_turn_angle_control", "trajectory_duration", 1)
vertiq4.set("multi_turn_angle_control", "trajectory_angular_displacement", math.pi*3/2)
vertiq4.set("multi_turn_angle_control", "trajectory_duration", 1)

time.sleep(1)


# 207.66 TARGET SEA LEVEL
for x in range(100):
    for index, vertiq in enumerate(vertiqs):
        vertiq.set("multi_turn_angle_control", "ctrl_velocity", x)
    time.sleep(.02)

print("Ready")
while True:
    for index, vertiq in enumerate(vertiqs):
        randDiff = random.uniform(0, 3)
        vertiq.set("multi_turn_angle_control", "ctrl_velocity", 100 - 1.5 + randDiff)
    time.sleep(.3)