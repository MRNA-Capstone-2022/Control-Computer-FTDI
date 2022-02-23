import iqmotion as iq
import time
import math

# Setup com ports (Subject to change based on where USB is plugged in)
com1 = iq.SerialCommunicator("/dev/cu.usbserial-14410")
com2 = iq.SerialCommunicator("/dev/cu.usbserial-14420")
com3 = iq.SerialCommunicator("/dev/cu.usbserial-14430")
com4 = iq.SerialCommunicator("/dev/cu.usbserial-14440")

# VARIABLES
elapsedTime = 0.0, time = 0.0, timePrev = 0.0
rad_to_deg = 180/math.pi
PID = 0, PID1 = 0, PID2 = 0, PID3 = 0
updateangle = 0.0, desired_angle = 0.0
angle = 0.0
est_angle0 = 0.0, est_angle1 = 0.0, est_angle2 = 0.0, est_angle3 = 0.0
error0 = 0.0, error1 = 0.0, error2 = 0.0, error3 = 0.0
pid_p_0 = 0.0, pid_i_0 = 0.0, pid_d_0 = 0.0
pid_p_1 = 0.0, pid_i_1 = 0.0, pid_d_1 = 0.0
pid_p_2 = 0.0, pid_i_2 = 0.0, pid_d_2 = 0.0
pid_p_3 = 0.0, pid_i_3 = 0.0, pid_d_3 = 0.0
est_diff = 0.0, diff = 0.0, diff2 = 0.0
time0 = 0.0, time1 = 0.0, time2 = 0.0, time3 = 0.0
prevPID_0 = 0.0, prevPID_1 = 0.0, prevPID_2 = 0.0, prevPID_3 = 0.0
newvel = 0.0, obs_vel = 0.0

# MOTOR SPEED SET
motorRads = 100.0

# --- PID CONSTANT SET -------------------------------------------
kp=1;   # kp = 1 for 100rad/s, kp = 4 for 650rad/s 
ki=2;   # ki = 2 for 100rad/s, ki = 8 for 650rad/s
kd=0;   # Derivative was not used

# Initialize motors as IQ objects
motor1 = iq.Vertiq2306(com1, 0)
motor2 = iq.Vertiq2306(com2, 0)
motor3 = iq.Vertiq2306(com3, 0)
motor4 = iq.Vertiq2306(com4, 0)
motors = [motor1, motor2, motor3, motor4]

# Store initial time
time = time.time()

# Set initial velocity
for motor in motors:
    motor.set("propeller_motor_control", "ctrl_velocity", motorRads)

def PID_Function(serialPort, motorRads, pid_p, pid_i, pid_d, prevPID, error, motorTime, est_angle, offset):

    desired_angle = 





while True:



#motor.set("propeller_motor_control", "ctrl_velocity", 1400)
#print(motor.get("brushless_drive", "obs_velocity"))