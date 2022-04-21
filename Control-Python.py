import iqmotion as iq
import time
import math

# Setup com ports (Subject to change based on where USB is plugged in)
com1 = iq.SerialCommunicator("/dev/cu.usbserial-144410")
com2 = iq.SerialCommunicator("/dev/cu.usbserial-144420")
com3 = iq.SerialCommunicator("/dev/cu.usbserial-144430")
com4 = iq.SerialCommunicator("/dev/cu.usbserial-144440")

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
startTime = time.time()

# Set initial velocity
for motor in motors:
    motor.set("propeller_motor_control", "ctrl_velocity", motorRads)

def hmodRadf(h):
    dh = 0.0
    i = 0

    if (h > 0):
        i = int(h/(2*math.pi)+0.5)
    else:
        i = int(h/(2*math.pi)-0.5)
    dh = h - math.pi*2*i

    return dh

def PID_Function(motor, motorRads, pid_p, pid_i, pid_d, prevPID, error, motorTime, est_angle, offset):
    # get the angle of the virtual reference
    desired_angle = hmodRadf(motorRads*(time/1000))

    # get the angle of actual motor
    currentAngle = motor.get("brushless_drive", "obs_angle")

    # find the difference between motor and "ref"
    # offset 0: motor is parallel to the virtual motor
    # offset PI/2: motor is 90 degrees off with respect to virtual motor
    diff = hmodRadf(desired_angle - angle + offset); 

    # new estimate angle
    """ 1. propogate 
    //      angle += (measured rpm)*elapsedTime
    // 2. update  
    //      estimated diff = estimated angle - desired angle
    // update = ((diff-est_diff)->[-PI to PI])*((small_gain)->[0-1]))
    // est_diff += update
    // angle -= update """
    currentVelocity = motor.get("brushless_drive", "obs_velocity")
    est_angle += currentVelocity*motorTime               # estimate the angle of the motor using time since last PID call
    est_angle = hmodRadf(est_angle)                      # set est_angle between [-Pi, Pi]
  
    est_diff = desired_angle - est_angle + offset   # estimate the difference between the desired and estimated angle
    est_diff = hmodRadf(est_diff)                       # set est_diff between [-Pi, Pi]
  
    updateangle = hmodRadf(diff-est_diff)*0.1
    est_diff += updateangle
    est_angle -= updateangle
    *est_angle = hmodRadf(*est_angle);

    // Calculating P, I, D
    *pid_p = est_diff*kp;
    *pid_i += est_diff*(*motortime);
    //*pid_d = (motorRads - obs_vel)*kd; //not in use

    // Final PID 
    PID = *pid_p + ki*(*pid_i) + *pid_d;
    
    // Calculate the new velocity
    newvel = motorRads*1.04 + PID;  // gain value 1.04 puts real motor speed closest to the virtual motor
    
    // Command new velocity to motor
    serialPort.set(prop.ctrl_velocity_,newvel);
    
    // Save current PID value for future PID
    *prevPID = PID;
    
    // Save current error for future PID
    *error = est_diff;



while True:



#motor.set("propeller_motor_control", "ctrl_velocity", 1400)
#print(motor.get("brushless_drive", "obs_velocity"))