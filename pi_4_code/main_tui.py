# Main function for pi 4
from typing import Any
from threading import Thread
import serial, time, signal, math, sys, os
from perception_and_drive import path_planning, robot_class_pi4, ball_search_drive
from pico_serial import pico_serial
from tui_functions import tui
import numpy as np
import pandas as pd
from gpiozero import DistanceSensor
import cv2

import sys
import io

try:
    import __builtin__
except ImportError:
    # Python 3
    import builtins as __builtin__

# define instance of robot
global robot, check_angle 
robot = robot_class_pi4.DiffDriveRobot()

check_angle = False
print_enabled = False

# define variables
global return_to_box, no_tennis, turn_on_rollers
return_to_box = False
no_tennis = 0
turn_on_rollers = False


print_to_log = True

if print_to_log:
    sys.stdout = io.open('output.log', mode='w')
    print("starting main")
    sys.stdout.flush()

# Serial initialisation --> ensure motors start at standstill
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
ser.reset_input_buffer()
reset_str = str("RES")
ser.write(reset_str.encode())

#def print(*args, **kwargs):
#    if print_enabled:
#        __builtin__.print(*args, **kwargs)
#    else:
#        window1obj.addstr(26, 1, *args) 
#    return



def reverse_backwards_calc(robot, ser, box_x, box_y, max_speed=1, turn_factor = 1):
    #sensor = DistanceSensor(echo = 18, trigger = 17)
    #dist_box = sensor.distance
    dx = box_x - robot.x
    dy = box_y - robot.y
    distance = np.sqrt(dx**2 + dy**2)

    while distance > 0.05:
        #dist_box = sensor.distance
        dx = box_x - robot.x
        dy = box_y - robot.y
        distance = np.sqrt(dx**2 + dy**2)
        v_desired = 0 
        w_desired = 0
        wheel_radius = 0.05
        wheel_sep = 0.15

        # calculating distance bewteen target and robot
        dx = box_x - robot.x
        dy = box_y - robot.y
        distance = np.sqrt(dx**2 + dy**2)
        print(f"distance: {distance:.2f}\n")

        # Calculate required rotation of the robot to face the target
        angle_to_target = math.atan2(dy, dx)    # from x_axis
        
        robot_theta = robot.th + np.pi
        if robot_theta < 0:
            robot_theta = robot_theta + 2*math.pi
        elif robot_theta > 2*math.pi:
            robot_theta = robot_theta - 2*math.pi    
        
        # THIS IS THE ONE FOR THE REAL ROBOT 
        rotation_angle = robot_theta - angle_to_target
        
        # adjust rotation angle to choose acute angle equivalent 
        if abs(rotation_angle) > np.pi:
            rotation_angle = 2*np.pi - abs(rotation_angle)
    
        #print(f"check angle is being changed here = {check_angle}\n")
        # Limit max velocity the closer we get to target, and limit at a quicker rate as we are closer than 75 cm.
        ### CAN BE TUNED to help stop overshooting the target

        print("reached correct angle and now driving straight\n")
        print("reached correct2.59 angle and now driving straight\n")

        if distance > 2:
            v_desired = max_speed * (distance / (distance + 2))
            #if v_desired < 0.55:
            #    v_desired = 0.55
        else:
            #v_desired = max_speed * (distance / (distance + 3))
            #print(f"HELLO lookie here - we are entienr this place {v_desired} ")
            v_desired = max_speed * (distance / (distance + 5))

        # Limit the turn rate to no more than +/- 5 rad/s
        if (turn_factor * rotation_angle) < 0:
            #w_desired = max(turn_factor * rotation_angle, -5)
            w_desired = max(5*turn_factor * rotation_angle, -5)
        elif (turn_factor * rotation_angle) > 0:
            #w_desired = min(turn_factor * rotation_angle, 5)
            w_desired = min(5*turn_factor * rotation_angle, 5)
        else:
            w_desired = 0

        print(f"v_desired: {v_desired:.2f}, w_desired: {w_desired:.2f}\n")

        # calculating wheel rotation speeds (rad/sec)
        wl = ( -v_desired + wheel_sep * w_desired/2 ) / wheel_radius
        wr = ( -v_desired - wheel_sep * w_desired/2 ) / wheel_radius
        print(f"required wl: {wl:.2f}, required wr: {wr:.2f}\n")

        ccc = 30000
        grad = 800

        # adjusting the values to be good for the input to PWM
        if wl < 0:
            #wl = -(10000 + abs(wl) * ((30000 - 10000) / 25))
            wl = -ccc + grad * wl
            print(f"we are changing wl new =  {wl}  \n")
        else:
            #wl = (10000 + abs(wl) * ((30000 - 10000) / 25))
            wl = ccc + grad * wl
            print(f"we are changing wl new =  {wl}  \n")
        if wr < 0:
            #wr = -(10000 + abs(wr) * ((30000 - 10000) / 25))
            wr = -ccc + grad * wr
            print(f"we are changing wr new =  {wr}  \n")
        else: 
            #wr = (10000 + abs(wr) * ((30000 - 10000) / 25))
            wr = ccc + grad * wr
            print(f"we are changing wr new =  {wr}  \n")
        
        time.sleep(0.1)
        
        print(f"new wl and wr = {wl} and {wr}\n")

        # set both motor speeds --> !! dir 0 vs dir 1 what does it mean !!
        # using sign of wlr
        if wl >= 0:
            robot.drive_wheel(ser, abs(wl), direction = "forwards", wheel = "left")
            #print(f"setting left wheel forwards speed {wl}\n\n")

        elif wl < 0:
            robot.drive_wheel(ser, abs(wl), direction = "backwards", wheel = "left")
            #print(f"setting left wheel backwards {wl}\n\n")

        if wr >= 0:    
            robot.drive_wheel(ser, abs(wr), direction = "forwards", wheel = "right")
            #print(f"setting right wheel forwards {wr}\n\n")

        elif wr < 0:
            robot.drive_wheel(ser, abs(wr), direction = "backwards", wheel = "right")
            #print(f"setting right wheel backwards {wr} \n\n")

        else:
            print("Serial not connected :(")  



def reverse_to_box(robot, court_width, court_length,box_position):
    print("Test return to box")

    tui.display_current_function(window1obj, "return_to_box") 

    # spin robot so the back is facing directly to the box
    if box_position[0] == 0 and box_position[1] == 0:
        targ_ang = np.pi/4
    elif box_position[0] > 0 and box_position[1] == 0:
        targ_ang = 3*np.pi/4
    elif box_position[0] == 0 and box_position[1] > 0: 
        targ_ang = 7*np.pi/4
    elif box_position[0] > 0 and box_position[1] > 0: 
        targ_ang = 5*np.pi/4

    req_rot_angle = abs(robot.th - targ_ang)
    angle_tol = 0.17 ## TUNEABLE
    while robot.th > targ_ang + angle_tol or robot.th < targ_ang - angle_tol:
        print("ROBOT TURNING\n")
        if req_rot_angle > 0:
            wl = -40000
            robot.drive_wheel(ser, abs(wl), direction = "backwards", wheel = "left")
            wr = 40000
            robot.drive_wheel(ser, abs(wr), direction = "forwards", wheel = "right")
        else:
            wl = 40000
            robot.drive_wheel(ser, abs(wl), direction = "forwards", wheel = "left")
            wr = -40000
            robot.drive_wheel(ser, abs(wr), direction = "backwards", wheel = "right")
        print(f"theta = {robot.th}, x {robot.x}, y = {robot.y}\n")

    robot.stop_motors(ser)

    # Option to reverse based on pose
    reverse_backwards_calc(robot, ser, box_position[0], box_position[1], max_speed=1, turn_factor = 1)
    
    # Option to reverse based on distance sensor
    # reverse drive robot along y-axis until 2cm away from the box 
    # TODO will need to update the GPIO values for echo and trigger 
    #print("Before sensor declaration")
    #sensor = DistanceSensor(echo=18, trigger=17)
    #dist_box = sensor.distance
    
    #while dist_box > 0.1: # TUNEABLE
    #    dist_box = sensor.distance
    #    print("ROBOT REVERSE\n")
    #    print(f"Distance to box: {dist_box}\n")
    #    #time.sleep(1)
    #    scale_down = 1.7
    #    run_speed = 40000
    #    wl = run_speed/scale_down
    #    robot.drive_wheel(ser, abs(wl), direction = "backwards", wheel = "left")
    #    wr = run_speed
    #    robot.drive_wheel(ser, abs(wr), direction = "backwards", wheel = "right")

    #    print(f"theta = {robot.th}, x {robot.x}, y = {robot.y}\n")
   
    robot.stop_motors(ser)
    
    # Drop tray flap to release balls
    robot.servo_open(ser)
    print("servo open")
    time.sleep(5)
    robot.servo_close(ser)

def drive_to_target(window1obj, window2obj, robot, target = [10, 0], dist_encode = 10, threshold = 0.05, return_to_box = False):
    #global robot
    global check_angle
    # Run drive to target function
    check_angle = False


    tui.display_current_function(window1obj, "drive_to_target")
    
    tui.update_robot_coords(window1obj, robot)

    global stop_threads
    stop_threads = False

    while  dist_encode > threshold:
        # call function to set wheel speeds
        #robot_position_update()
        _,_, angle, check_angle = path_planning.set_wheel_speeds(ser, robot, check_angle, target = [target[0], target[1]])
        
#        robot.drive_wheel(ser, 15000, direction = "forwards", wheel = "right")
#        robot.drive_wheel(ser, 15000, direction = "forwards", wheel = "left")
#        angle = 0

        # update robot position and recalc the distance
        dist_encode = np.sqrt( (target[0] - robot.x)**2 + (target[1] - robot.y)**2 )
        angle_deg = np.rad2deg(angle)


        window1obj.addstr(24, 1, f"angle: {angle_deg}") 
        print(f"new distance is = {dist_encode}\n\n")


        # DRAW ON TUI
        tui_x, tui_y = robot.x/court_width*100, robot.y/court_length*100

        window1obj.addstr(23, 1, f"dist: {dist_encode}") 
        
        tui.draw_robot(window2obj, tui_x, tui_y)
        tui_targx, tui_targy = target[0]/court_width*100, target[1]/court_length*100
        # TODO need to limit the tennis ball drawing to prevent it from putting a ball over the top of the box - not crucial, just for the tui
        tui.draw_tennis_ball(window2obj, tui_targx, tui_targy)
        tui.update_robot_coords(window1obj, robot)
        #window1obj.refresh()
        #window2obj.refresh()
        window1obj.noutrefresh()
        window2obj.refresh()

        #stdscr .doupdate()
        window1obj.nodelay(True)
        c = window1obj.getch()
        if c!= -1 and c == ord('q'):
           tui.quit_tui()
           robot.stop_motors(ser)
           stop_threads = True
           os._exit(1)
           return  # Exit the while loop
 
    print(f"BEFORE return to box {return_to_box}\n")
    if return_to_box == False:
        # Drive forwards for a few seconds to run over ball
        print("Driving forwards")
        robot.stop_motors(ser)
        robot.drive_wheel(ser, 30000, direction = "forwards", wheel = "left")
        robot.drive_wheel(ser, 30000, direction = "forwards", wheel = "right")
        time.sleep(1) ## If you make this 1 it's super accurate, but I think we want the overshoot
        robot.stop_motors(ser)
    else:
        print("Robot is at the BOX")
        # drive to a position close to the box
        #reverse_to_box(robot,court_length,court_width, box_position) # TODO I think this is circular now which may be a problem
        

    # direction - last direct


def traverse_circle(robot, window1obj, window2obj, width, length, targ_box_x, targ_box_y, box_position):
    while 1:
        nudge = 0.6
        direction = "BR"
        targ_x, targ_y = width - 0.2, 0.2
        dist_length = length - nudge
        dist_width = width - nudge

        dist_encode = np.sqrt( (targ_x - robot.x)**2 + (targ_y - robot.y)**2 )
        tennis_threshold = 0.15 
        drive_to_target(window1obj, window2obj, robot, [targ_x, targ_y], dist_encode, tennis_threshold, return_to_box)
        targ_array = [[0,0], [targ_x, targ_y]]
    
        break_condition = False
        for i in range(60): 
    
            # check if it should keep driving along x axis and if its left or right
            if direction == "BR":
                targ_y = targ_y + dist_length
                dist_length = dist_length - nudge
                direction = "TR"
                dist_encode = np.sqrt( (targ_x - robot.x)**2 + (targ_y - robot.y)**2 )
                tennis_threshold = 0.15 
                drive_to_target(window1obj, window2obj, robot, [targ_x, targ_y], dist_encode, tennis_threshold, False)
                targ_array.append([targ_x, targ_y])
    
            elif direction == "TR":
                targ_x = targ_x - dist_width
                dist_width = dist_width - nudge
                direction = "TL"
                dist_encode = np.sqrt( (targ_x - robot.x)**2 + (targ_y - robot.y)**2 )
                tennis_threshold = 0.15 
                drive_to_target(window1obj, window2obj, robot, [targ_x, targ_y], dist_encode, tennis_threshold, False)
                targ_array.append([targ_x, targ_y])
    
            elif direction == "TL":
                targ_y = targ_y - dist_length
                dist_length = dist_length - nudge
                direction = "BL"
                dist_encode = np.sqrt( (targ_x - robot.x)**2 + (targ_y - robot.y)**2 )
                tennis_threshold = 0.15 
                drive_to_target(window1obj, window2obj, robot, [targ_x, targ_y], dist_encode, tennis_threshold, False)
                targ_array.append([targ_x, targ_y])
            
            elif direction == "BL":
                targ_x = targ_x + dist_width
                dist_width = dist_width - nudge
                direction = "BR"
                dist_encode = np.sqrt( (targ_x - robot.x)**2 + (targ_y - robot.y)**2 )
                tennis_threshold = 0.15 
                drive_to_target(window1obj, window2obj, robot, [targ_x, targ_y], dist_encode, tennis_threshold, False)
                targ_array.append([targ_x, targ_y])
    
            if break_condition == True: 
                break
            
            if dist_length < 0 or dist_width < 0: 
                break_condition = True 
            
        dist_encode = np.sqrt( (targ_box_x - robot.x)**2 + (targ_box_y - robot.y)**2 )
        tennis_threshold = 0.15 
        drive_to_target(window1obj, window2obj, robot, [targ_x, targ_y], dist_encode, tennis_threshold, False)
        reverse_to_box(robot,length,width, box_position)
    return targ_array


def convert_robot_world_coord(tx_robot,ty_robot):
    # function converts the tennis ball coordinates from the robot to the world frame axis 
    # tx_robot = the tennis ball x coord from the robot frame, rx_origin = the robot x coord from the frame frame, robot = robot class 

    tx_world = tx_robot*np.cos(robot.th) - ty_robot*np.sin(robot.th) + robot.x
    ty_world = tx_robot*np.sin(robot.th) + ty_robot*np.cos(robot.th) + robot.y

    return tx_world, ty_world


def search_for_tennis_ball(robot):
    
    print("starting search for ball \n")
    # initialise ball searching locations 
    search_drive_locations = [[1.03, 1.07], [1.03, 3.2], [1.03, 5.33], [3.08, 1.07], [3.08, 3.2], [3.08, 5.33]]              # coords for if it can reach 2m radius [2.05, 2.13], [2.05, 4.27]]
    
    # do an intial search at start location 
    #coord = ball_search_drive.init_search(robot)
    #x = coord[0][0]
    #y = coord[0][1]
    #dist = np.sqrt((x-robot.x)**2 + (y-robot.y)**2)
    coord = None 
    while len(search_drive_locations) != 0:   
        # find the closest point in the search_drive_locations 
        min_distance = 1000
        for idx, point in enumerate(search_drive_locations): 
            cur_distance = np.sqrt((point[0] - robot.x)**2 + (point[1] - robot.y)**2)
            if cur_distance < min_distance: 
                min_distance = cur_distance
                closest_point = point
                closest_idx = idx
        print(f"driving to target {closest_point}\n")
        drive_to_target(window1obj, window2obj, robot, [closest_point[0], closest_point[1]], cur_distance, 0.15, False)
        search_drive_locations.pop(closest_idx)
        

        print(f"starting camera stuff\n")
        coord = ball_search_drive.init_search(robot, ser)
        if coord is not None:
            x = coord[0][1]
            y = coord[0][1]
            dist = np.sqrt((x - robot.x)**2 + (y - robot.y)**2)
            return dist, x, y
        
        
    return -1, -1, -1   

    


loop = True
while loop:
    
    # Important known constants
    
    # actual box coords 
    court_length = 4.05 -0.4#m
    court_width = 5.40 -0.4 #m

    #court_length = 6.40 # m
    #court_width = 4.11 # m
    
    ball_diam = 6.3 # cm
    #box_position = [court_width, court_length] # [x,y]
    box_position = [1,1]
    #box_position = [1,1] # CHANGE

    # define TUI
    window1obj, window2obj = tui.screen_init()

    tui.display_current_function(window1obj, "main")
    tui_boxx, tui_boxy = (box_position[0] - 0.05)/court_width * 100, (box_position[1] - 0.05)/court_length*100
    tui.draw_box(window2obj,tui_boxx,tui_boxy)
    window2obj.refresh()

    serial_thread = Thread(target=pico_serial.robot_update_background, args=(ser, robot))
    serial_thread.daemon = True
    serial_thread.start()

    # Run drive to target function
    tennis_threshold = 0.10 

    # adjusting the target box coordinates to drive to a little distance from the box to allow for turning room 
    if box_position[0] == 0:
        box_targ_x = 0.6
    else:
        box_targ_x = box_position[0] - 0.6
    if box_position[1] == 0:
        box_targ_y = 0.6
    else:
        box_targ_y = box_position[1] - 0.6


    # REMOVE THIS 
    #dist_encode = np.sqrt( (1 - robot.x)**2 + (1 - robot.y)**2 )
    #drive_to_target(window1obj, window2obj, robot, [0, 1], dist_encode, 0.15, return_to_box)
    ####################################################################################################################################
    ####################################################################################################################################
    ####################################################################################################################################
    # uncomment this if you want to disable drive as a last resort
    #traverse_circle(robot, window1obj, window2obj, court_width, court_length, box_targ_x, box_targ_y, box_position)
    # ^^ will run tennis ball collection and then reverse to box and then will run loop again
    ####################################################################################################################################
    ####################################################################################################################################
    ####################################################################################################################################

    # SEARCH BALL DRIVE
    # loop to find 3-5 balls and then ends loop and runs return to box function 
    #dist, ten_x, ten_y = search_for_tennis_ball(robot)
    
    #targs_coords = [[box_targ_x, box_targ_y]]
    
    # Comment back in this section Gretty
    #for i in range(5):
    #    dist, ten_x, ten_y = search_for_tennis_ball(robot)
    #    if dist < 0:
    #        break
    #    tennis_threshold = 0.1
    #    dist_encode = np.sqrt( (targ_x - robot.x)**2 + (targ_y - robot.y)**2 )
    #    drive_to_target(window1obj, window2obj, robot, [ten_x, ten_y], dist_encode, tennis_threshold, return_to_box)

    tennis_threshold = 0.1
    #dist_encode = np.sqrt( (box_targ_x - robot.x)**2 + (box_targ_y - robot.y)**2 )
    #drive_to_target(window1obj, window2obj, robot, [box_targ_x, box_targ_y], dist_encode, tennis_threshold, return_to_box=True)
    # GGB hardcoding to test driving
    dist = 100
    ten_x, ten_y = [0,1]
    if dist > 0: 
        #targs_coords = [[ten_x, ten_y],[box_targ_x, box_targ_y]] # TODO replace with camera coords
        targs_coords = [[box_targ_x,box_targ_y]]
    else: 
        #if no ball was found then we need to go straight to the box, as it should have searched the whole court 
        targs_coords = [[box_targ_x, box_targ_y]]
    no_tennis = 0

    for targ in targs_coords:
        if len(targs_coords) - no_tennis == 1:   # Return to box as we have reached final coord in target array
            return_to_box = True

        targ_x,targ_y = targ
        robot.stop_motors(ser)
        dist_encode = np.sqrt( (targ_x - robot.x)**2 + (targ_y - robot.y)**2 )

    
        drive_to_target(window1obj, window2obj, robot, [targ_x, targ_y], dist_encode, tennis_threshold,return_to_box)
        print_enabled = True
        print(f"Robot has attempted to drive to ball at coords {targ_x},{targ_y} at distance {dist_encode}\n")
        print(f"Robot itself is at coords {robot.x},{robot.y}\n")
        
        
        
        # TOO include functions to turn on roller motors here

        # Stop motors once arrived
        robot.stop_motors(ser)
        print("Arrived at target")
        no_tennis = no_tennis + 1
        print(f"Number of acquired tennis balls is: {no_tennis}")

    # TODO return to box once balls acquired
    #drivie_to_box(robot, court_width, court_length, box_position)
    reverse_to_box(robot,court_length,court_width, box_position)
    
    c = window1obj.getch()
    if c!= -1 and c == ord('q'):
       tui.quit_tui()
       robot.stop_motors(ser)
       os._exit(1)

    #sys.stdout.close()
    tui.quit_tui()
    
    # to run loop ONCE!!!!!
    loop = True
