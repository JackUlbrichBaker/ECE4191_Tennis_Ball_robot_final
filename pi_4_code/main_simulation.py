# Main function for pi 4
from typing import Any
import serial, time, signal, math
from perception_and_drive import path_planning, robot_class_pi4
import numpy as np
import pandas as pd

# imports for simulation
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import math
import random
from IPython.display import display
from IPython.display import clear_output


# define axes and graph variables
plt.ion()
fig, (ax, ax2) = plt.subplots(1, 2, figsize=(12, 8))
ax.set_xlim(-1, 5)
ax.set_ylim(-1, 7)
ax.set_aspect('equal')
ax.grid(True)
ax2.set_xlabel('Frame Number')
ax2.set_ylabel('Rotation Speed (rad/s)')
ax2.set_ylim(-10, 10)
x_data = []
wl_data = []
wr_data = []
e_sum_l, e_sum_r = 0, 0
plt.show(block=False)

# define instance of robot
global robot, check_angle
robot = robot_class_pi4.DiffDriveRobot(inertia=5, dt=0.1, drag=0.2, wheel_radius=0.05, wheel_sep=0.15)
check_angle = False

# define variables
global return_to_box, no_tennis, turn_on_rollers
return_to_box = False
no_tennis = 0
turn_on_rollers = False
global ten_x, ten_y
ten_x, ten_y = 0, 0


# # Serial initialisation --> ensure motors start at standstill
# ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
# ser.reset_input_buffer()
# reset_str = str("RES")
# stop_str_1 = "PWM 0 0 0  \n\r "
# stop_str_2 = "PWM 1 0 0\n\r"
# ser.write(reset_str.encode())


# def read_commands(ser):         # Function to read from the USB serial stream to get data from pi pico
#     # inputs: ser (serial object)
#     # outputs: line (string)

#     line = ser.readline().decode('utf-8').rstrip()
#     time.sleep(0.01)
#     print("input string: ", line)
#     return line

# def update_robot(ser_data):      # Function that accepts data from the pi pico in the form of: "\nx: 0.12343 \ny: 1.2345 \nt: 90" and updates current robot position
#     # inputs:  ser_data (string)
#     # outputs: robot.update_x(x), robot.update_y(y), robot.update_t(theta) (float)

#     if ser_data is None:
#         print("no valid data") # early exit if no valid data
#         return

#     coord = ser_data[0:2]       # read x, y, t (theta)
#     value = pd.to_numeric(ser_data[2:-1])
#     if coord == "x:":
#         try:
#             x = float(value)
#         except:
#             return
#         robot.update_x(x)

#     elif coord == "y:":
#         try:
#             y = float(value)
#         except:
#             return
#         robot.update_y(y)

#     elif coord == "t:":
#         try:
#             theta = float(value)
#         except:
#             return
#         robot.update_theta(theta)

# def foobar(): # Function reads serial data x3 to ensure data is captured correctly
#     for i in range(3):
#         ser_data = read_commands(ser)       # read commands
#         update_robot(ser_data)               # update robot pose

# def stop_motor():
#     ser.write(stop_str_1.encode())
#     time.sleep(0.1)
#     ser.write(stop_str_2.encode())
#     ser.write(stop_str_2.encode())
#     ser.write(stop_str_2.encode())
#     time.sleep(5)


#### START OF ECE4191 RESOURCES --> simulation of robot drive
def p_control(w_desired,w_measured,e_sum):
    Kp=0.1
    Ki=0.01
    duty_cycle = min(max(-1,Kp*(w_desired-w_measured) + Ki*e_sum),1)

    e_sum = e_sum + (w_desired-w_measured)

    return duty_cycle, e_sum

# gets required duty cucle from required wheel speeds
def drive(req_wl,req_wr,wl,wr):
    global e_sum_l, e_sum_r
    duty_cycle_l, e_sum_l = p_control(req_wl, wl, e_sum_l)
    duty_cycle_r, e_sum_r = p_control(req_wr, wr, e_sum_r)

    return duty_cycle_l, duty_cycle_r

# gets actual wheel speeds
def motor_simulator(w,duty_cycle):

    I = 10
    dt = 0.1
    d = 1

    torque = I*duty_cycle

    if (w > 0):
        w = min(w + dt*(torque - d*w),10)
    elif (w < 0):
        w = max(w + dt*(torque - d*w),-10)
    else:
        w = w + dt*(torque)

    return w


def update_robot_sim():
    dt=0.1
    wheel_radius = 0.05
    wheel_sep = 0.15

    v = (robot.wl*wheel_radius + robot.wr*wheel_radius)/2.0

    w = (robot.wl*wheel_radius - robot.wr*wheel_radius)/wheel_sep

    robot.x = robot.x + dt*v*np.cos(robot.th)
    robot.y = robot.y + dt*v*np.sin(robot.th)
    robot.th = robot.th + w*dt

    if robot.th < 0:
        robot.th = robot.th + 2*np.pi
    elif robot.th > 2*np.pi:
        robot.th = robot.th - 2*np.pi


### end of ECE4191 resources


def drive_to_target(robot, target = [10, 0], dist_encode = 10, threshold = 0.05):
    global check_angle
    # Run drive to target function
    check_angle = False
    update_robot_sim()
    while  dist_encode > threshold:
        # call function to set wheel speeds
        ser = 0
        req_wl, req_wr = path_planning.set_wheel_speeds(ser, robot, check_angle, [target[0], target[1]]) # theoretically we don't need direction eventually

        # simulate robot driving at calculated required speeds and get new wheel speeds
        dcl, dcr = drive(req_wl, req_wr, robot.wl, robot.wr)
        robot.wl = motor_simulator(robot.wl, dcl)
        robot.wr = motor_simulator(robot.wr, dcr)

        # update robot position and recalc the distance
        #foobar()

        # update robot posey through simulation
        update_robot_sim()

        dist_encode = np.sqrt( (target[0] - robot.x)**2 + (target[1] - robot.y)**2 )


        # update graph on new posey
        ax.clear()
        ax.set_xlim(-1, 5)
        ax.set_ylim(-1, 7)
        ax.set_aspect('equal')
        ax.grid(True)

        # add tennis ball markers
        ax.plot(ten_x, ten_y, 'go', markersize=5)

        # add robot
        ax.plot(robot.x, robot.y, 'bo', markersize=6)

        # add robot arrow
        end_x = robot.x + 0.3 * np.cos(robot.th)
        end_y = robot.y + 0.3 * np.sin(robot.th)
        ax.arrow(robot.x, robot.y, end_x - robot.x, end_y - robot.y, head_width=0.07, head_length=0.07, fc='r', ec='r')

        ax.set_title(f"Robot Position: ({robot.x:.2f}, {robot.y:.2f}), Orientation: {np.degrees(robot.th):.2f}°")

        if len(x_data) > 0:
            last_x = x_data[-1]
        else:
            last_x = -1
        x_data.append(last_x+1)
        wl_data.append(robot.wl)
        wr_data.append(robot.wr)

        # Limit data to last 20 frames (assuming 1 data point per second)
        if len(x_data) > 40:
            x_data.pop(0)
            wl_data.pop(0)
            wr_data.pop(0)

        # Plot data
        ax2.clear()
        left = ax2.plot(x_data, wl_data, color='blue')
        right = ax2.plot(x_data, wr_data, color='red')


        ax2.set_xlabel('Frame Number')
        ax2.set_ylabel('Rotation Speed (rad/s)')
        ax2.legend([left, right], ['Left Wheel', 'Right Wheel'], loc='upper right')
        ax2.set_ylim(-12, 12)

        plt.draw()
        plt.pause(0.005)








def return_to_box(robot, court_width, court_length, box_position):
    print("driving to box\n\n")

    # drive to a position close to the box
    if box_position[0] == 0:
        targ_x = 0.10
    else:
        targ_x = court_length - 0.10 # GGB swapped length and width

    if box_position[1] == 0:
        targ_y = 0.10
    else:
        targ_y = court_width - 0.10 ## GGB swapped length and width

    print(f"targ x = {targ_x} and targ y = {targ_y}\n\n\n\n")

    current_dis = np.sqrt( (targ_x - robot.x)**2 + (targ_y - robot.y)**2 )
    drive_to_target(robot, [targ_x, targ_y], current_dis, threshold = 0.05) # GGB suggestion - could we not just threshold it here? instead of the if/else statements above?

    # spin robot so the back is facing directly to the box
    # GGB changed court_length and court_width to be box_position[0] and box_position[1]
    if box_position[0] == 0 and box_position[1] == 0:
        targ_ang = np.pi/4
    elif box_position[0] > 0 and box_position[1] == 0:
        targ_ang = 3*np.pi/4
    elif box_position[0] == 0 and box_position[1] > 0:
        targ_ang = 7*np.pi/4
    elif box_position[0] > 0 and box_position[1] > 0:
        targ_ang = 5*np.pi/4

    req_rot_angle = abs(robot.th - targ_ang)
    angle_tol = 0.3 # GGB added because theta is not perfect - TUNEABLE
    while robot.th > targ_ang + angle_tol or robot.th < targ_ang - angle_tol:

        if abs(req_rot_angle) > np.pi:
            req_rot_angle = 2*np.pi - abs(req_rot_angle)

        req_wheel_rotate = req_rot_angle * robot.l/2
        if req_rot_angle > 0:
            robot.wl = motor_simulator(robot.wl, req_wheel_rotate)
            robot.wr = motor_simulator(robot.wr, -req_wheel_rotate)
            # GGB - Don't need to stop the motors because the while loop condition does it
            # time.sleep(1)
            # robot.wl = motor_simulator(robot.wl, 0)
            # robot.wr = motor_simulator(robot.wr, 0)
        else:
            robot.wl = motor_simulator(robot.wl, -req_wheel_rotate)
            robot.wr = motor_simulator(robot.wr, req_wheel_rotate)
            # GGB - Don't need to stop the motors because the while loop condition does it
            # time.sleep(1)
            # robot.wl = motor_simulator(robot.wl, 0)
            # robot.wr = motor_simulator(robot.wr, 0)

        # GGB copied this code in from drive code - might be worth writing a function
        # for this seeing as it's an exact copy but also probs not worth it just for the simulation I guess?

        update_robot_sim()
        ax.clear()
        ax.set_xlim(-1, 5)
        ax.set_ylim(-1, 7)
        ax.set_aspect('equal')
        ax.grid(True)

        # add tennis ball markers
        ax.plot(ten_x, ten_y, 'go', markersize=5)

        # add robot
        ax.plot(robot.x, robot.y, 'bo', markersize=6)

        # add robot arrow
        end_x = robot.x + 0.3 * np.cos(robot.th)
        end_y = robot.y + 0.3 * np.sin(robot.th)
        ax.arrow(robot.x, robot.y, end_x - robot.x, end_y - robot.y, head_width=0.07, head_length=0.07, fc='r', ec='r')

        ax.set_title(f"Robot Position: ({robot.x:.2f}, {robot.y:.2f}), Orientation: {np.degrees(robot.th):.2f}°")

        if len(x_data) > 0:
            last_x = x_data[-1]
        else:
            last_x = -1
        x_data.append(last_x+1)
        wl_data.append(robot.wl)
        wr_data.append(robot.wr)

        # Limit data to last 20 frames (assuming 1 data point per second)
        if len(x_data) > 40:
            x_data.pop(0)
            wl_data.pop(0)
            wr_data.pop(0)

        # Plot data
        ax2.clear()
        left = ax2.plot(x_data, wl_data, color='blue')
        right = ax2.plot(x_data, wr_data, color='red')


        ax2.set_xlabel('Frame Number')
        ax2.set_ylabel('Rotation Speed (rad/s)')
        ax2.legend([left, right], ['Left Wheel', 'Right Wheel'], loc='upper right')
        ax2.set_ylim(-12, 12)

        plt.draw()
        plt.pause(0.005)


    # reverse drive robot along y-axis until 2cm away from the box
    # TODO will need to update the GPIO values for echo and trigger
    #sensor = DistanceSensor(echo=18, trigger=17)

    dist_box = np.sqrt( (box_position[0] - robot.x)**2 + (box_position[1] - robot.y)**2 )
    while dist_box > 0.02:
        #dist_box = sensor.distance * 1
        print('Distance to box: ', dist_box)
        time.sleep(1)

        # drive robot straight ahead
        robot.wl = motor_simulator(robot.wl, 5)
        robot.wr = motor_simulator(robot.wr, 5)
        dist_box = np.sqrt( (box_position[0] - robot.x)**2 + (box_position[1] - robot.y)**2 )


    robot.wl = motor_simulator(robot.wl, 0)
    robot.wr = motor_simulator(robot.wr, 0)
    # run motor to release flap for tennis balls
    # TODO call this function
    # delay, and run motor to put flap back in place


while True:
    # Important known constants
    box_position = np.array([4, 6]) # [x,y]
    court_width = 6.4 # m - changed GGB
    court_length = 4.11 # m - changed GGB
    ball_diam = 0.063 # m - changed GGB

    # Return to box if counter/sensor says we should --> set condition
    # TODO could add sensor condition here as well
    if (return_to_box == True or no_tennis == 1):
        # update robot pos + run drive function to box coordinates
        #foobar()

        #update_robot_sim()

        # dist_box = np.sqrt( (box_position[0] - robot.x)**2 + (box_position[1] - robot.y)**2 )
        # box_threshold = 0.2
        # print("driving to box")
        # drive_to_target([box_position[0], box_position[1]], dist_box, box_threshold)
        # print("arrived at box")
        # # Stop motors
        # #stop_motor()
        # TODO: call function to release tennis balls


        return_to_box(robot, court_width, court_length, box_position)

        # reset values
        no_tennis = 0
        return_to_box = False

    # call functions to locate tennis ball
    # retry finding tennis ball until camera distance + encoder distance are close --> tune threshold
    # stop_motor()
    # dist_vision = 0
    # dist_encode = 1
    # while dist_vision/dist_encode <= 0.95 or dist_vision/dist_encode >=1.05:
    #     # Run function to find tennis balls --> returns distance and x,y coords of ball
    #     dist_vision, ten_x, ten_y = path_planning.ball_search_drive.init_search()
    #     dist_encode = np.sqrt( (ten_x - robot.x)**2 + (ten_y - robot.y)**2 )

    print(f"starting tennis ball collecting again!")

    # get random x y coord for tennis ball
    ten_x = random.uniform(0, 4)
    ten_y = random.uniform(0, 6)
    dist_encode = np.sqrt( (ten_x - robot.x)**2 + (ten_y - robot.y)**2 )

    # Run drive to target function
    tennis_threshold = 0.05
    drive_to_target(robot, [ten_x, ten_y], dist_encode, tennis_threshold)

    # TOO include functions to turn on roller motors here

    # Stop motors once arrived
    #stop_motor()
    print(f"Arrived at target - {no_tennis} \n\n")
    no_tennis = no_tennis + 1
