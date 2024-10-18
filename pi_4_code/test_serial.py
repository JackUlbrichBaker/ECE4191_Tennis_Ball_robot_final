import serial, time, main
#from perception_and_drive import *
import numpy as np
import pandas as pd

# define instance of robot
global robot, angle
robot = robot_class_pi4.DiffDriveRobot(inertia=5, dt=0.1, drag=0.2, wheel_radius=0.05, wheel_sep=0.15)
check_angle = False


def read_commands(ser):         # Function to read from the USB serial stream to get data from pi pico
    # inputs: ser (serial object)
    # outputs: line (string)

    line = ser.readline().decode('utf-8').rstrip()
    time.sleep(0.01)
    print("input string: ", line)
    return line

def update_robot(ser_data):      # Function that accepts data from the pi pico in the form of: "\nx: 0.12343 \ny: 1.2345 \nt: 90" and updates current robot position
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

    elif coord == "y:":
        try:
            y = float(value)
        except:
            return
        robot.update_y(y)

    elif coord == "t:":
        try:
            theta = float(value)
        except:
            return
        robot.update_theta(theta)
