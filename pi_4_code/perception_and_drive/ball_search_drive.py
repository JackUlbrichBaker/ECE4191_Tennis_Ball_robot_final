# Written by Kelly, last updated 04/10/24

import cv2
import numpy as np
import pandas as pd
#from flask import Flask, Response

from .ball_detect import detect_single_ball_cv2, detect_single_ball
import io
import sys

print_to_log = False

if print_to_log:
    sys.stdout = io.open('output.log', mode='w')
    print("hello")
    sys.stdout.flush()


##### NOT IN USE  #####
def calculate_distance(real_radius, image_radius, focal_length, x_pixel, camera_height, camera_angle):
    # Calculate the distance using the formula
    tx_robot = x_pixel * (real_radius/(2*image_radius))                      #distance = (real_radius * focal_length) / image_radius

    # get distance from robot to ball
    measured_distance = (real_radius * focal_length ) / (2*image_radius)     # distance = (known width of tennis ball * focal length) / pixel width
    
    # y coord without the camera angle 
    #ty_robot = np.sqrt(measured_distance**2 - camera_height**2)
    # y coord with the camera angle 
    ty_robot = np.sqrt(measured_distance**2 + camera_height**2) * np.cos(np.radians(camera_angle))


    # TODO add equations to account for camera angle!

    return measured_distance, tx_robot, ty_robot

def get_ball_distance(detected_circle):
    f_x = 656.7164179104477  # Camera intrinsic parameter: Focal length in x direction (pixels)
    R_real = 0.0335  # Real-world radius of ball in meters
    camera_height = 0.08   # TODO get actual camera height - should this be in metres?
    camera_angle = np.pi()/4

    x, y, r =  detected_circle
    # Calculate distance to the ball
    distance, x, y = calculate_distance(R_real, r, f_x, x, camera_height, camera_angle)

    print(f"The distance to the ball is: {distance} and coords in robot frame: ({x}, {y})")

    return distance, x, y
##### #####

### CORRECT CONVERTING FUNCTIONS ###

def convert_pixels_robot_coord(x_pixel, radius):
    # function converts the pixel counts to the x distance and y distance from the robot in cm
    # x_pixel = hoziontal pixel number from centre of image; radius = radius of circle drawn around tennis ball
    # source : https://pyimagesearch.com/2015/01/19/find-distance-camera-objectmarker-using-python-opencv/
    img_centre = (640-1)//2
    avg_tennis_diameter = ( (6.54 + 6.86) /2 ) / 100 # m
    scale_factor = avg_tennis_diameter / (2*radius) # m/pixel
    focal_length = 656.7164179104477           # need to calibrate focal length
    camera_height = 4 / 100                             # (m) TODO need to measure camera height

    # calculate horizontal distance in cm from directly ahead of camera
    tx_robot = abs(img_centre - x_pixel) * scale_factor
    if x_pixel < img_centre:
        tx_robot = -tx_robot

    # print(f"tx_robot = {tx_robot}")

    # get distance from robot to ball
    measured_distance = (avg_tennis_diameter * focal_length ) / (2*radius)     # distance = (known width of tennis ball * focal length) / pixel width
    # print(f"measured distance: {measured_distance}")
    ground_distance = np.sqrt(measured_distance**2 - camera_height**2)
    print(f"Distance measured (m): {ground_distance}")
    ty_robot = np.sqrt(ground_distance**2 - tx_robot**2)

    return tx_robot, ty_robot

def convert_robot_world_coord(tx_robot,ty_robot, robot):
    '''
    function converts the tennis ball coordinates from the robot to the world frame axis   
    tx_robot (FLOAT) = the tennis ball x coord from the robot frame  
    ty_robot (FLOAT) = the tennis ball x coord from the robot frame  
    robot_pose (1x3 array) = robot class ####NOT - pose of current robot [x, y, theta]  
    '''
    x_pose, y_pose, theta_pose = robot.x, robot.y, robot.th

    tx_world = tx_robot*np.cos(theta_pose) - ty_robot*np.sin(theta_pose) + x_pose
    ty_world = tx_robot*np.sin(theta_pose) + ty_robot*np.cos(theta_pose) + y_pose

    return tx_world, ty_world
    #return tx_robot, ty_robot

def check_ball_bounds(world_coords):
    '''
    INPUT: world_coords (2D ARRAY) = n x 3 array where n = number of balls detected  
    OUTPUT: 2D array of filtered coords
    '''
    print(f"Checking that coordinates are within bounds...")
    #print(f"Original array size = {len(world_coords[0])}")

    # actual length 
    court_length = 4.05 #m
    court_width = 5.40 #m

    #court_length = 6.40 # m
    #court_width = 4.11 # m
    good_tennis_coords = []

    for coord in world_coords:
        ten_x, ten_y = coord
        # TODO depending on how accurate our camera is we should add some leniency here 🙂
        if ten_x >= 0 and ten_x <= court_width and ten_y >= 0 and ten_y <= court_length:
            # add coordinate to good list
            good_tennis_coords.append(coord)

    if good_tennis_coords == []:
        print("No detected balls are within bounds...")
        return None
    else:
        #print(f"Final array size = {len(good_tennis_coords[0])}")
        return good_tennis_coords



def init_search(robot, ser):
    print("Initiating search for ball...")

    final_circle = None

    # TODO add condition to specify the angles array based on robot posey 
    angles = np.array([0, 60, 60, 60, 60, 60]) # array of angles for robot to rotate at, relative to robot frame
    angles = angles * np.pi/180
    # TODO add condition for angles arrays to decide which direction to rotate depending on position on court (esp corners vs centre)
    ball_found = False
    angle_idx = 0

    for angle in angles: 
        # pivoting to correct angle!
        
        start_robot_ang = robot.th # take note of initial angle because we only need to rotate relative to current robot orientation
        print(f"start robot ang condition = {abs(robot.th - start_robot_ang)} and condition {abs(robot.th - start_robot_ang) < angle} and ANGLE = {angle}\n")
        while abs(robot.th - start_robot_ang) < angle:
            print("ROBOT TURNING in init search \n")
            wl = 40000
            robot.drive_wheel(ser, abs(wl), direction = "forwards", wheel = "left")
            wr = -40000
            robot.drive_wheel(ser, abs(wr), direction = "backwards", wheel = "right")

        robot.stop_motors(ser)
        
        print("scanning for ball\n")
        # scanning for a ball! 
        final_circle = detect_single_ball_cv2()

        # convert coords to good coords 
        if final_circle is not None: 
            x, y, radius = final_circle 
            print(f"PIXELS from final circle x = {x}, radius {radius}\n")
            tx_robot, ty_robot = convert_pixels_robot_coord(x, radius)
            print(f"before adjustements ROBOT FRAME x = {tx_robot}, y = {ty_robot}\n")
            tx_robot = tx_robot - 0.1565 # in (m) - accounts for camera positioned to left of collection mechanism
            ty_robot = ty_robot + 0.36 # m
            print(f"ROBOT FRAME x = {tx_robot}, y = {ty_robot}\n")
            tx_world, ty_world = convert_robot_world_coord(tx_robot, ty_robot, robot)
            print(f"WORLD FRAME x = {tx_world}, y = {ty_world}\n")
            
            # check boundaries 
            coords = check_ball_bounds([[tx_world, ty_world]])


        # determine if a ball was found
            if coords is not None:
                return coords
            
            
        else:
            print("No ball detected at this angle")
            
    # if we have looped through and no coord found
    return None


    ############## MAIN BALL SEARCH LOOP ################
    #while ball_found == False:

        # Search for ball
    #    angle_idx = 0
        
        ############ SEARCH SEQUENCE ############
      #  while final_circle is None: # While ball not detected
            # pivot_to_search(robot, angles[angle_idx],ser)
            # final_circle = ball_detect.detect_ball(view_mode = True)

            # Pivot robot
      #      print(f"Pivoting at angle: {angles[angle_idx]}")

      #      # Look for a ball
      #      start_picam()
      #      final_circle = detect_ball_picam()
      #      print(f"BALL FOUND! Final circle: {final_circle} \n")

      #      angle_idx+=1

            # If no ball is detected in robot's current position, drive elsewhere
      #      if angle_idx > len(angles)-1:
      #          angle_idx = 0
      #          print("No ball detected in area. Driving elsewhere...")
                # TODO: drive forwards
                # go back to main function to drive 
                
        ############ DRIVE TO BALL ############

        # If final_circle returns a valid array:        
     #   ball_found = True


    # Ball is found, measure the distance
    # coords in robot frame 
    #print("Getting distances to ball...")
    # TODO: get distances
    #dist, x, y = 1, 1, 1
    # dist, x, y = get_ball_distance(final_circle)

    # check dist is greater than 0 
    #if dist <= 0 or dist == None: 
        #return -1,-1,-1
    
    #return dist,x,y

# TODO: THREADING

# Initialising Flask
#app = Flask(__name__)

#@app.route('/video_feed')
#def video_feed():
#    return Response(detect_ball_picam(),
#                    mimetype='multipart/x-mixed-replace; boundary=frame')

#@app.route('/')
#def index():
#    return "Welcome to the Camera Stream! Go to /video_feed to view the feed."

#if __name__ == "__main__":
#    app.run(host='0.0.0.0', port=5000)  # Make it accessible on your network
#    init_search()
