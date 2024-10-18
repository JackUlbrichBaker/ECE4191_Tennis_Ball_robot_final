##  Helper functions to talk to robot over serial connections
import serial, time
import pandas as pd

def read_commands(ser):
    # Function to read from the USB serial stream to get data from pi pico
    # inputs: ser (serial object)
    # outputs: line (string)

    line = ser.readline().decode('utf-8').rstrip()
    time.sleep(0.01)
    #if line != "":
        #print("input string: ", line)
    return line

def robot_update_background(ser, robot):
    # Will update robot position if date is recieved correctly, otherwise it will do nothing
    while 1:
        #robot_position_update(ser, robot)
        try:
            robot_position_update(ser, robot)
        except:
            pass

        time.sleep(0.01)

def robot_position_update(ser, robot):
    # Function reads serial data three times to ensure data is captured correctly
    x_change, y_change, t_change = False, False, False
    count = 0    
    while (x_change == False or y_change == False or t_change == False) and count < 4:
        ser_data = read_commands(ser)
        x_change, y_change, t_change = update_robot(robot, ser_data,x_change,y_change,t_change) # update robot pose
        count = count + 1

def update_robot(robot, ser_data,x_change,y_change,t_change):
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

def test_threads(ser, robot):
    while 1:
        x = robot.x
        y = robot.y
        robot.update_x(x+0.1)
        robot.update_y(y+0.047)
        time.sleep(0.5)

