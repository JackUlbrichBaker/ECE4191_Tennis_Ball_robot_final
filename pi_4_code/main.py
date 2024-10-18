# Main function for pi 4
from typing import Any
from threading import Thread
import serial, time, signal, math
from perception_and_drive import path_planning, robot_class_pi4
from pico_serial import pico_serial

import numpy as np
import pandas as pd
from gpiozero import DistanceSensor
print("TEST")
# define instance of robot
global robot, check_angle 
robot = robot_class_pi4.DiffDriveRobot()

check_angle = False
print_enabled = True

# define variables
global return_to_box, no_tennis, turn_on_rollers
return_to_box = False
no_tennis = 0
turn_on_rollers = False


# Serial initialisation --> ensure motors start at standstill
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.01)
ser.reset_input_buffer()
reset_str = str("RES")
stop_str_1 = "PWM 0 0 0  \n\r "
stop_str_2 = "PWM 1 1 0\n\r"
ser.write(reset_str.encode())


try:
    import __builtin__
except ImportError:
    # Python 3
    import builtins as __builtin__

def print(*args, **kwargs):
    __builtin__.print(*args, **kwargs)


def drive_to_target(robot, target = [1, 0], threshold = 0.05):
    #global robot
    #global check_angle
    # Run drive to target function
    check_angle = False

    dist_encode = np.sqrt( (target[0] - robot.x)**2 + (target[1] - robot.y)**2 )

    while  dist_encode > threshold:
        # call function to set wheel speeds
        #foobar()
        time.sleep(0.4)
        path_planning.set_wheel_speeds(ser, robot, check_angle, [target[0], target[1]]) # theoretically we don't need direction eventually
        
        print("drive forwards")
        #robot.drive_wheel(ser, 65535, direction = "forwards", wheel = "left")
        #robot.drive_wheel(ser, 65535, direction = "forwards", wheel = "right")

        # update robot position and recalc the distance
        dist_encode = np.sqrt( (target[0] - robot.x)**2 + (target[1] - robot.y)**2 )
        print(f"new distance is = {dist_encode}\n\n")
        print(f"Robot coords currently are: x: {robot.x}, y: {robot.y}, t: {robot.th}")

def return_to_box(robot, court_width, court_length):
    #print("Test return to box")

    # drive to a position close to the box 
    if court_width == 0:
        targ_x = court_width + 10
    else:
        targ_x = court_width - 10
    
    if court_length == 0:
        targ_y = court_length + 10 
    else: 
        targ_y = court_length - 10 

    current_dis = np.sqrt( (targ_x - robot.x)**2 + (targ_y - robot.y)**2 )
    drive_to_target(robot, [targ_x, targ_y], current_dis, threshold = 0.05)

    # spin robot so the back is facing directly to the box
    if court_length == 0 and court_width == 0:
        targ_ang = np.pi/4
    elif court_length > 0 and court_width == 0:
        targ_ang = 7*np.pi/4
    elif court_length == 0 and court_width > 0: 
        targ_ang = 3*np.pi/4
    elif court_length > 0 and court_width > 0: 
        targ_ang = 5*np.pi/4

    
    req_rot_angle = abs(robot.th - targ_ang)
    if abs(rotation_angle) > np.pi:
        rotation_angle = 2*np.pi - abs(rotation_angle)


    req_wheel_rotate = req_rot_angle * robot.l/2
    if req_rot_angle > 0:
        string = "PWM 0 0 " + str(req_wheel_rotate) + "\n"
        ser.write(string.encode())
        string = "PWM 1 1 " + str(req_wheel_rotate) + "\n"
        ser.write(string.encode())
        time.sleep(1)
        ser.write(stop_str_1.encode())
        ser.write(stop_str_1.encode())
        ser.write(stop_str_1.encode())
        time.sleep(0.1)
        ser.write(stop_str_2.encode())
        ser.write(stop_str_2.encode())
        ser.write(stop_str_2.encode())
    else:
        string = "PWM 0 1 " + str(req_wheel_rotate) + "\n"
        ser.write(string.encode())
        string = "PWM 1 0 " + str(req_wheel_rotate) + "\n"
        ser.write(string.encode())
        time.sleep(1)
        ser.write(stop_str_1.encode())
        ser.write(stop_str_1.encode())
        ser.write(stop_str_1.encode())
        time.sleep(0.1)
        ser.write(stop_str_2.encode())
        ser.write(stop_str_2.encode())
        ser.write(stop_str_2.encode())

    # reverse drive robot along y-axis until 2cm away from the box 
    # TODO will need to update the GPIO values for echo and trigger 
    sensor = DistanceSensor(echo=18, trigger=17)
    while dist_box > 0.02:
        dist_box = sensor.distance * 1
        print('Distance to box: ', dist_box)
        time.sleep(1)
        
        # drive robot straight ahead
        motor0 = "VEL 0 0 10000\n"
        ser.write(motor0.encode())
        motor1 = "VEL 1 0 10000\n"
        ser.write(motor1.encode())
    
    robot.stop_motors(ser)
    
    # run motor to release flap for tennis balls 
    # TODO call this function 
    # delay, and run motor to put flap back in place 


def convert_robot_world_coord(tx_robot,ty_robot):
    # function converts the tennis ball coordinates from the robot to the world frame axis 
    # tx_robot = the tennis ball x coord from the robot frame, rx_origin = the robot x coord from the frame frame, robot = robot class 

    tx_world = tx_robot*np.cos(robot.th) - ty_robot*np.sin(robot.th) + robot.x
    ty_world = tx_robot*np.sin(robot.th) + ty_robot*np.cos(robot.th) + robot.y

    #return tx_world, ty_world
    return tx_robot, ty_robot

def drive_distance():
    # Drives a specified distance
    return

def pivot(degree):
    # Pivots
    return

############################################ MAIN LOOP ############################################################
print("Initialization complete")

while True:

    print("starting drive loop")
    # Important known constants
    court_length = 6.40 # m
    court_width = 4.11 # m
    ball_diam = 6.3 # cm
    box_position = np.array([[court_width, court_length]]) # [x,y]

    serial_thread = Thread(target=pico_serial.robot_update_background, args=(ser, robot))
    serial_thread.daemon = True
    serial_thread.start()

    # Return to box if counter/sensor says we should --> set condition
    # TODO could add sensor condition here as well
    # if (return_to_box == True or no_tennis == 5):
    #     # update robot pos + run drive function to box coordinates
    #     foobar()
    #     dist_box = np.sqrt( (box_position[0] - robot.x)**2 + (box_position[1] - robot.y)**2 )
    #     box_threshold = 0.2
    #     drive_to_target(robot,[box_position[0], box_position[1]], dist_box, box_threshold)
    #     # Stop motors
    #     stop_motor()
    #     # TODO: call function to release tennis balls

    #     # reset values
    #     no_tennis = 0
    #     return_to_box = False

    # # call functions to locate tennis ball
    # # retry finding tennis ball until camera distance + encoder distance are close --> tune threshold
    # stop_motor()
    # dist_vision = 0
    # dist_encode = 1
    # #back_list = []
    # while dist_vision/dist_encode <= 0.95 or dist_vision/dist_encode >=1.05 or dist_vision <= 0:
    #     print(f"Ratio: {dist_vision/dist_encode}\n")
    #     print(f"Vision dist: {dist_vision}\n")
    #     if dist_vision == -1:
    #         # there are no balls we can see from this robot position 
    #         # drive to center of court
    #         box_centre = [2,3]
    #         dist_to_centre = np.sqrt( (box_centre[0] - robot.x)**2 + (box_centre[1] - robot.y)**2)
    #         threshold = 0.05
    #         drive_to_target(robot,box_centre, dist_box, threshold)
             
    #     # Run function to find tennis balls --> returns distance and x,y coords of ball
    #     # dist_vision, ten_x, ten_y = ball_search_drive.init_search(robot)
    #     dist_vision = 1
    #     ten_x = 2
    #     ten_y = 2
    #     print("Robot knows where ball is")

    #     # Convert ten_x, ten_y to world frame 
    #     ten_x, ten_y = convert_robot_world_coord(ten_x, ten_y)
        
    #     # Check detected ball is within tennis court
    #     if ten_x >0 and ten_x < court_width and ten_y >0 and ten_y < court_length:
    #         # TODO tolerance for court dimensions
    #         dist_encode = np.sqrt( (ten_x - robot.x)**2 + (ten_y - robot.y)**2 )
    #     else: 
    #         print("Tennis ball outside court bounds")
    #         dist_vision = 0
    #         dist_encode = 1
    #         # add tennis ball coords to black list!
    #         #black_list.append([ten_x, ten_y]) 
            
    # Run drive to target function
    tennis_threshold = 0.05
    ten_x = 2
    ten_y = 0.3
    robot.stop_motors(ser)
     
    
    #dist_encode = np.sqrt( (ten_x - robot.x)**2 + (ten_y - robot.y)**2 )
    drive_to_target(robot, [ten_x, ten_y], tennis_threshold)
    #print(f"Robot has attempted to drive to ball at coords {ten_x},{ten_y} at distance {dist_encode}\n")
    #print(f"Robot is currently at position x: {robot.x}, y: {robot.y}, t: {robot.th}\n")

    # TOO include functions to turn on roller motors here

    # Stop motors once arrived
    robot.stop_motors(ser)
    #print("Arrived at target")
    no_tennis = no_tennis + 1



