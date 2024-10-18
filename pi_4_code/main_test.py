# kelly testing code 19/09/24
# deleted commented code but retained most of main.py structure

# Main function for pi 4
from typing import Any
from threading import Thread
import serial, time, signal, math
from perception_and_drive import path_planning, ball_search_drive, robot_class_pi4, drive
import numpy as np
import pandas as pd
# from gpiozero import DistanceSensor

############################################ FUNCTIONS ############################################################

def read_commands(ser):
    # Function to read from the USB serial stream to get data from pi pico
    # inputs: ser (serial object)
    # outputs: line (string)

    line = ser.readline().decode('utf-8').rstrip()
    time.sleep(0.01)
    #print("input string: ", line)
    return line

def robot_update_background(ser, robot):
    # Will update robot position if date is recieved correctly, otherwise it will do nothing
    while 1:
        try:
            robot_position_update(ser, robot)
        except:
            pass
        time.sleep(0.1)

def robot_position_update(ser, robot):
    # Function reads serial data three times to ensure data is captured correctly
    x_change, y_change, t_change = False, False, False
    count = 0    
    while (x_change == False or y_change == False or t_change == False) and count < 4:
        ser_data = read_commands(ser)
        x_change, y_change, t_change = update_robot(ser_data,x_change,y_change,t_change) # update robot pose
        count = count + 1

def update_robot(ser_data,x_change,y_change,t_change):
    # Function that accepts data from the pi pico in the form of: "\nx: 0.12343 \ny: 1.2345 \nt: 90" and updates current robot position
    # inputs:  ser_data (string)
    # outputs: robot.update_x(x), robot.update_y(y), robot.update_t(theta) (float)

    if ser_data is None:
        print("no valid data") # early exit if no valid data
        return

    coord = ser_data[0:2]       # read x, y, t (theta)
    value = pd.to_numeric(ser_data[2:-1])
    if coord == "x:":
        try:
            x = float(value)
        except:
            return
        robot.update_x(x)
        x_change = True
        #print("x changed")

    elif coord == "y:":
        try:
            y = float(value)
        except:
            return
        robot.update_y(y)
        y_change = True
        #print("y changed")

    elif coord == "t:":
        try:
            theta = float(value)
        except:
            return 
        robot.update_theta(theta)
        t_change = True
        #print("t changed")

    return x_change, y_change, t_change

def stop_motor():
    ser.write(stop_str_1.encode())
    time.sleep(0.1)
    ser.write(stop_str_2.encode())
    ser.write(stop_str_2.encode())
    ser.write(stop_str_2.encode())
    time.sleep(5)

def drive_to_target(robot, target = [2, 0], dist_encode = 2, threshold = 1):
    """
    dist encode = meters
    """
    #global robot
    #global check_angle
    # Run drive to target function
    check_angle = False

    while  dist_encode > threshold:
        # call function to set wheel speeds
        #foobar()
        time.sleep(1)
        #path_planning.set_wheel_speeds(ser, robot, check_angle, [target[0], target[1]]) # theoretically we don't need direction eventually
        
        print("Driving forwards...")
        motor0 = "PWM 0 1 65535\n"
        ser.write(motor0.encode())
        motor1 = "PWM 0 0 65535\n"
        ser.write(motor1.encode())

        # update robot position and recalc the distance
        dist_encode = np.sqrt( (target[0] - robot.x)**2 + (target[1] - robot.y)**2 )
        print(f"New distance is = {dist_encode}\n\n")
        print(f"Robot coords currently are: x: {robot.x}, y: {robot.y}, t: {robot.th}")

############################################ KELLY CODE ############################################################
def calculate_dist_from_encoder(robot, encoder_counts):
    gear_ratio = 75
    counts_per_revolution = 48
    pulses_per_revolution = counts_per_revolution * gear_ratio
    wheel_diameter = robot.r*2  # Example wheel diameter in meters
    wheel_circumference = 3.14 * wheel_diameter  # Circumference = π * diameter

    revolutions = encoder_counts / pulses_per_revolution
    distance_meters = revolutions * wheel_circumference

    return distance_meters

def drive_straight_distance(robot, target_x_coord):
    '''
    coords (meters) = x
    TODO: adapt this code to take in x and y inputs
    '''

    # overshoot_dist = 0.05 # drive 5cm over required distance to ensure ball is collected
    print(f"Initial Pose: xyz = {robot.x}, {robot.y}, {robot.th}")

    # Turn on motor
    print(f"Driving forwards by {target_x_coord} meters...")
    motor0 = "PWM 0 1 65535\n"
    ser.write(motor0.encode())
    motor1 = "PWM 0 0 65535\n"
    ser.write(motor1.encode())

    reached_flag = False
    threshold = 0.1

    # Loop until destination has been reached
    while reached_flag == False:

        time.sleep(2)

        start_time = time.time()
        timeout_duration = 5 # 10 seconds before it times out

        # Update pose
        update_robot_pose(robot)

        # Print pose
        current_x = robot.x
        current_y = robot.y
        current_theta = robot.th
        print(f"New Pose (x, y, theta) = ({current_x}, {current_y}, {current_theta})")

        # Check if reached target coordinates
        # TODO: change this part specifically, make sure that the current_x is relative to the target coordinate (from 0)
        if current_x + threshold >= target_x_coord:
            # Stop motors once arrived
            stop_motor()
            print("Arrived at target")

        elif time.time() - start_time > timeout_duration:
            stop_motor()
            print("Timeout, check for errors")

def update_robot_pose(robot):

    ser_data = ser.readline().decode('utf-8').rstrip()
    time.sleep(0.01)

    # Error handling
    if not ser_data:
        print("No valid data") # early exit if no valid data
        return None

    # Checking to see which position it has recieved an update from
    else:
        coord = ser_data[0:2]       # read x, y, t (theta)
        # print(f"Raw data: {coord}")

        value = pd.to_numeric(ser_data[2:-1])
        if coord == "x:":
            try:
                x = float(value)
                robot.update_x(x)
                print(f"pose x has been updated to = {x}")
                return x
            except:
                return 
        elif coord == "y:":
            try:
                y = float(value)
                robot.update_y(y)
                print(f"pose y has been updated to = {y}")
                return y
            except:
                return
        elif coord == "t:":
            try:
                theta = float(value)
                robot.update_theta(theta)
                print(f"pose theta has been updated to = {theta}")
                return theta
            except:
                return 
            
def pivot(degree):
    # Pivots
    return

############################################ INITIALISATION ############################################################

# Define instance of robot
global robot, check_angle 
robot = robot_class_pi4.DiffDriveRobot(inertia=5, dt=0.1, drag=0.2, wheel_radius=0.0265, wheel_sep=0.2775)

# measure values with calipers!
check_angle = False
print_enabled = True

stop_str_1 = "PWM 0 0 0  \n\r "
stop_str_2 = "PWM 1 1 0\n\r"

try:
    import __builtin__
except ImportError:
    # Python 3
    import builtins as __builtin__

# Serial initialisation --> ensure motors start at standstill
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.01)
ser.reset_input_buffer()
reset_str = str("RES")
ser.write(reset_str.encode())

def print(*args, **kwargs):
    if print_enabled:
        __builtin__.print(*args, **kwargs)
    return


############################################ TEST drive funcs 

drive.pivot_to_angle(robot, 45)
while 1:
    time.sleep(10)

############################################ MAIN LOOP ############################################################

# Important known constants
court_length = 6.40 # m
court_width = 4.11 # m
ball_diam = 6.3 # cm
box_position = np.array([[court_width, court_length]]) # [x,y]

# Serial initialisation
serial_thread = Thread(target=robot_update_background, args=(ser, robot))
serial_thread.daemon = True
serial_thread.start()
        
# Run drive to target function
distance_threshold = 0.5
target_x_coord = 1
target_y_coord = 0

x_dists = [1,1.5]
for dist in x_dists:

    # Ensure motor is standstill
    stop_motor()

    print("Initialising drive sequence...")
    drive_straight_distance(robot, dist)
    
print("yay")
