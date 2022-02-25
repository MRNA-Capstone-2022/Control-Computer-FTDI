from tracemalloc import start
import iqmotion as iq
import time
import math

# Setup com ports (Subject to change based on where USB is plugged in)
com1 = iq.SerialCommunicator("/dev/cu.usbserial-14110")
com2 = iq.SerialCommunicator("/dev/cu.usbserial-14120")
com3 = iq.SerialCommunicator("/dev/cu.usbserial-14130")
com4 = iq.SerialCommunicator("/dev/cu.usbserial-14140")

# Initialize motors as IQ objects
motor1 = iq.Vertiq2306(com1, 0, firmware="servo")
motor2 = iq.Vertiq2306(com2, 0, firmware="servo")
motor3 = iq.Vertiq2306(com3, 0, firmware="servo")
motor4 = iq.Vertiq2306(com4, 0, firmware="servo")
motors = [motor1, motor2, motor3, motor4]

# Store start time of Program
startTime = time.time()

# Target Speed (rad/s)
targetSpeed = 300

# Start Angles (How the blades were put on)
startMotor1 = -0.6
startMotor2 = 0.145

def hmodRadf(h):
    dh = 0.0
    i = 0

    if (h > 0):
        i = int(h/(2*math.pi)+0.5)
    else:
        i = int(h/(2*math.pi)-0.5)
    dh = h - math.pi*2*i

    return dh

# Path selection for code to run (0 to set position, 1 to trajectory)
path = 3


if path == 0:
    # Set the trajectory for the motor to complete 1 full rotation
    motor4.set("multi_turn_angle_control", "trajectory_angular_displacement", 0)
    motor3.set("multi_turn_angle_control", "trajectory_angular_displacement", 0)
    motor2.set("multi_turn_angle_control", "trajectory_angular_displacement", 0)
    motor1.set("multi_turn_angle_control", "trajectory_angular_displacement", 0)


    # Sets trajectory duration for 2 seconds
    motor4.set("multi_turn_angle_control", "trajectory_duration", 5)
    motor3.set("multi_turn_angle_control", "trajectory_duration", 5)
    motor2.set("multi_turn_angle_control", "trajectory_duration", 5)
    motor1.set("multi_turn_angle_control", "trajectory_duration", 5)
    time.sleep(1)

if path == 1:

    motor4.set("multi_turn_angle_control", "angular_speed_max", 300)
    motor3.set("multi_turn_angle_control", "angular_speed_max", 300.2)
    motor2.set("multi_turn_angle_control", "angular_speed_max", 300.5)
    motor1.set("multi_turn_angle_control", "angular_speed_max", 300.8)

    # Set the trajectory for the motor to complete 1 full rotation
    motor4.set("multi_turn_angle_control", "trajectory_angular_displacement", 500*math.pi)
    motor3.set("multi_turn_angle_control", "trajectory_angular_displacement", 500*math.pi)
    motor2.set("multi_turn_angle_control", "trajectory_angular_displacement", 500*math.pi)
    motor1.set("multi_turn_angle_control", "trajectory_angular_displacement", 500*math.pi)


    # Sets trajectory duration for 2 seconds
    motor4.set("multi_turn_angle_control", "trajectory_duration", 5)
    motor3.set("multi_turn_angle_control", "trajectory_duration", 5)
    motor2.set("multi_turn_angle_control", "trajectory_duration", 5)
    motor1.set("multi_turn_angle_control", "trajectory_duration", 5)

    motorDist = [0.0,0.0,0.0]
    motor0 = 0
    while True:
        for x in range(len(motors)):
            obsRads = motors[x].get("multi_turn_angle_control", "obs_angular_velocity")
            if (obsRads is not None):
                if (x == 0):
                    motor0 = obsRads
                else:
                    motorDist[x-1] = obsRads - motor0
                print("Motor %s Velocity: %s" % (x, obsRads))

if path == 2:
    motor1PrevSpeed = 0
    #motor1.set("multi_turn_angle_control", "angular_speed_max", 110)
    while True:

        timeDiff = time.time() - startTime
        theoroRads = timeDiff * targetSpeed

        obsDisplacement = motor1.get("multi_turn_angle_control", "obs_angular_displacement")
        if (obsDisplacement is not None):
            if (obsDisplacement < theoroRads) :
                motor1PrevSpeed += (theoroRads - obsDisplacement)*.0001
                motor1.set("multi_turn_angle_control", "ctrl_velocity", motor1PrevSpeed)
            elif (obsDisplacement > theoroRads):
                motor1PrevSpeed += (theoroRads - obsDisplacement)*.0001
                motor1.set("multi_turn_angle_control", "ctrl_velocity", motor1PrevSpeed)
        print("Thero rads: %s Obs rads %s" % (theoroRads, obsDisplacement))


if path == 3:
    while True:
        time.sleep(.001)
        for motor in motors:

            timeDiff = time.time() - startTime
            theoroRads = timeDiff * targetSpeed

            motor.set("multi_turn_angle_control", "ctrl_angle", theoroRads)


















"""
motor1.set("multi_turn_angle_control", "ctrl_velocity", 100)
motor2.set("multi_turn_angle_control", "ctrl_velocity", 100)

time.sleep(30)

motor1.set("multi_turn_angle_control", "ctrl_brake")
motor2.set("multi_turn_angle_control", "ctrl_brake")
"""

"""
while True:

    timeDiff = time.time() - startTime
    theoroRads = hmodRadf(timeDiff * targetSpeed)

    #obsRads = motor1.get("brushless_drive", "obs_angle")
    if (obsRads is not None):
        # Calculat difference between angles
        diffRads = obsRads - theoroRads

        print("Theoro Rads: %s Obs Rads: %s Diff Rads: %s" % (theoroRads, obsRads, diffRads))

    if (timeDiff > 3):
        print("Yurr")
        time.sleep(1)
        motor1.set("multi_turn_angle_control", "ctrl_brake")
        motor2.set("multi_turn_angle_control", "ctrl_brake")
        break
    


while True:
    #motor.set("propeller_motor_control", "ctrl_velocity", 1400)
    obsRads = motor1.get("brushless_drive", "obs_angle")
"""
