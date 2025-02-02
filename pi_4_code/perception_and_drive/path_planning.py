import numpy as np
import math, serial, time
#from matplotlib import pyplot as plt
#from IPython import display

# UNCOMMENT WHEN WANTING TO PRINT TO TEXT FILE
import sys
import io

# Redirect stdout to a file
simulation = False

print_to_log = False

if print_to_log:
    sys.stdout = io.open('output.log', mode='w')
    print("jelo")
    sys.stdout.flush()

max_pivot_effort = 0.01

def calculate_desired_velocities(robot, check_angle, tennis_x, tennis_y, max_speed=1, turn_factor = 1):

    v_desired = 0
    w_desired = 0

    wheel_radius = 0.05
    wheel_sep = 0.15

    # calculating distance bewteen target and robot
    dx = tennis_x - robot.x
    dy = tennis_y - robot.y
    distance = np.sqrt(dx**2 + dy**2)
    #print(f"distance: {distance:.2f}\n")

   # Calculate required rotation of the robot to face the target
    angle_to_target = math.atan2(dy, dx)    # from x_axis
    #rotation_angle = robot.th - angle_to_target
    rotation_angle = angle_to_target - robot.th

    # adjust rotation angle to choose acute angle equivalent 
    if abs(rotation_angle) > np.pi:
        rotation_angle = 2*np.pi - abs(rotation_angle)

    # find ideal angle to start driving the robot straight
    ### CAN BE TUNED to spin less/more
    angle_condition = np.pi/4*(distance/(distance+1))
    
    #print(f"angle_condition for starting to drive straight: {angle_condition:.2f}, needed rotation_angle : {rotation_angle:.2f}\n")
    #print(f"robot theta {robot.th}, x = {robot.x}, y = {robot.y}")
      

    ## taken away this if statement - originalyl set to 0.75 and distance + 1
    
   #if distance > 2:
        #v_desired = max_speed * (distance / (distance + 2))
        #if v_desired < 0.55:
        #    v_desired = 0.55
    #else:

    if abs(rotation_angle) >= angle_condition and check_angle == False:
        # set speeds to spin
        ### CAN BE TUNED to spin faster
        #print("ROBOT TURNING\n")

        # real life works with this code 
        #v_desired = 0
        #w_desired = 1
        #v_desired = 0
        #w_desired = 2

        # this is not working 
        #if rotation_angle > 0:
        #    v_desired = 0
        #    w_desired = 1
        #else:
        #    v_desired = 0
        #    w_desired = -1
        if rotation_angle > 0:
            v_desired = 0
            w_desired = 0.5
        else:
            v_desired = 0
            w_desired = -0.5
    else:
        check_angle = True
        #print(f"check angle is being changed here = {check_angle}\n")
        # Limit max velocity the closer we get to target, and limit at a quicker rate as we are closer than 75 cm.
        ### CAN BE TUNED to help stop overshooting the target

        #print("ROBOT DRIVING STRAIGHT\n")
        
        if distance > 0:
            #v_desired = max_speed * (distance / (distance + 2))
            #if v_desired < 0.55:
            #    v_desired = 0.55
            v_desired = 0.02
        else:
            #v_desired = max_speed * (distance / (distance + 3))
            v_desired = max_speed * (distance / (distance + 5))

        # Limit the turn rate to no more than +/- 5 rad/s
        if (turn_factor * rotation_angle) < 0:
            #w_desired = max(turn_factor * rotation_angle, -5)
            w_desired = max(0.5*turn_factor * rotation_angle, -5)
        elif (turn_factor * rotation_angle) > 0:
            #w_desired = min(turn_factor * rotation_angle, 5)
            w_desired = min(0.5*turn_factor * rotation_angle, 5)
        else:
            w_desired = 0

    #print(f"v_desired: {v_desired:.2f}, w_desired: {w_desired:.2f}\n")

    # calculating wheel rotation speeds (rad/sec)
    req_wl = ( v_desired - wheel_sep * w_desired/2 ) / wheel_radius
    req_wr = ( v_desired + wheel_sep * w_desired/2 ) / wheel_radius
    #print(f"required wl: {req_wl:.2f}, required wr: {req_wr:.2f}\n")

    
    #if req_wl < 0 and abs(w_desired) !=1:
    #    req_wl = 0
    #if req_wr < 0 and abs(w_desired) !=1:
    #    req_wr = 0
    
    # tryng to adjust range 
    #req_wl = ((req_wl + 5) * (55535 + 11107) / 30) - 11107
    #req_wr = ((req_wr + 5) * (55535 + 11107) / 30) - 11107
    #print(f"new adjusutments for max PWM wl = {req_wl} and wr = {req_wr}")
    #print(f"new adjusutments for max PWM wl = {req_wl} and wr = {req_wr}")

    # adjusting the wheel speeds to be proportional maximum speeds of our wheels
    ### TO BE TUNED to the max speeds of our wheels
    #motor_max_speed = 55535
    motor_max_speed = 20000
    if abs(req_wl) > motor_max_speed or abs(req_wr) > motor_max_speed:
        if req_wl > req_wr:
            req_wr = req_wr / req_wl * motor_max_speed
            req_wl = motor_max_speed
            print(f"we are in wl > wr setting wl = {motor_max_speed}\n")
            #print(f"we are in wl > wr setting wl = {motor_max_speed}\n")
        elif req_wr > req_wl:
            #req_wl = req_wl / req_wr * motor_max_speed
            req_wl = req_wl / req_wr * motor_max_speed # TODO PUT IN LOG SCALE HERE!!!!!!!!!
            req_wr = motor_max_speed
            print(f"we are in wl < wr setting wr = {motor_max_speed}\n")
            #print(f"we are in wl < wr setting wr = {motor_max_speed}\n")
        else:
            req_wl = motor_max_speed
            req_wr = motor_max_speed
            print(f"we are in wl = wr setting wl = {motor_max_speed}\n")
            #print(f"we are in wl = wr setting wl = {motor_max_speed}\n")

    #print(f"motor adjusted to max 55535 required wl: {req_wl:.2f}, required wr: {req_wr:.2f}")

    return req_wl, req_wr, rotation_angle, check_angle

#def calculate_desired_velocities(robot, check_angle, tennis_x, tennis_y, max_speed=1, turn_factor = 1):

#    v_desired = 0
#    w_desired = 0
#
#    wheel_radius = 0.05
#    wheel_sep = 0.15
#    turn_factor = 1

    # calculating distance bewteen target and robot
#    dx = tennis_x - robot.x
#    dy = tennis_y - robot.y
#    distance = np.sqrt(dx**2 + dy**2)
#    print(f"distance: {distance:.2f}\n")


    # Calculate required rotation of the robot to face the target
#    angle_to_target = math.atan2(dy, dx)    # from x_axis
#    rotation_angle = robot.th - angle_to_target
    #rotation_angle = angle_to_target - robot.th

    # adjust rotation angle to choose acute angle equivalent 
#    if abs(rotation_angle) > np.pi:
#        rotation_angle = 2*np.pi - abs(rotation_angle)

    # find ideal angle to start driving the robot straight
    ### CAN BE TUNED to spin less/more
#    angle_condition = np.pi/4*(distance/(distance+3)) #* 0.75
#    print(f"angle_condition for starting to drive straight: {angle_condition:.2f}, needed rotation_angle : {rotation_angle:.2f}\n")
#    print(f"robot theta {robot.th}")
    #print(f"check angle = {check_angle}\n")
    #print(f"here is the output of the first half of the if statemnet - {abs(rotation_angle) >= angle_condition}")
    
    # check if the robot needs to spin more
#    check_angle = True
    #print(f"check angle is being changed here = {check_angle}\n")
    # Limit max velocity the closer we get to target, and limit at a quicker rate as we are closer than 75 cm.
    ### CAN BE TUNED to help stop overshooting the target
    
    # dont think this is good 
    # check the angle is still good 
    #if abs(rotation_angle) > 1:
    #   check_angle = False


#    print("reached correct angle and now driving straight\n")

    ## taken away this if statement - originalyl set to 0.75 and distance + 1
    
     #if distance > 2:
         #v_desired = max_speed * (distance / (distance + 2))
         #if v_desired < 0.55:
         #    v_desired = 0.55
     #else:

#    turn_scale_speed_factor = (w_desired / (w_desired + 5))
#    v_desired = max_speed * (distance / (distance + 5)) #* turn_scale_speed_factor
    
#    w_desired = (turn_factor * rotation_angle) #* (5 + distance)/distance


    #  Limit the w_desired to +/-max_turn_amount 
#    max_turn_amount = 10
#    w_desired = min(max_turn_amount, max(-max_turn_amount, w_desired))

#    if abs(rotation_angle) >= angle_condition and check_angle == False:
         # set speeds to spin
         ### CAN BE TUNED to spin faster
#        print("is not at correct angle, robot is turning\n")
        
        # real life works with this code 
        #v_desired = 0
        #w_desired = 2

        # this is not working 
#        if rotation_angle > 0:
#            v_desired = 0.01 * v_desired
#            w_desired = max_pivot_effort
#        else:
#            v_desired = 0.01 * v_desired
#            w_desired = -max_pivot_effort


#    print(f"v_desired: {v_desired:.2f}, w_desired: {w_desired:.2f}\n")

#    # calculating wheel rotation speeds (rad/sec)
#    req_wl = ( v_desired + wheel_sep * w_desired/2 ) / wheel_radius
#    req_wr = ( v_desired - wheel_sep * w_desired/2 ) / wheel_radius
#    print(f"required wl: {req_wl:.2f}, required wr: {req_wr:.2f}\n")

     #if req_wl < 0 and abs(w_desired) !=2:
      #   req_wl = 0
     #if req_wr < 0 and abs(w_desired) !=2:
         #req_wr = 0
     # tryng to adjust range 
     #req_wl = ((req_wl + 5) * (55535 + 11107) / 30) - 11107
     #req_wr = ((req_wr + 5) * (55535 + 11107) / 30) - 11107
#    print(f"new adjusutments for max PWM wl = {req_wl} and wr = {req_wr}")

    # adjusting the wheel speeds to be proportional maximum speeds of our wheels
    ### TO BE TUNED to the max speeds of our wheels
#    motor_max_speed = 200000000000000000000000000000000
#    if abs(req_wl) > motor_max_speed or abs(req_wr) > motor_max_speed:
#        if req_wl > req_wr:
#            req_wr = req_wr / req_wl * motor_max_speed
#            req_wl = motor_max_speed
            #print(f"we are in wl > wr setting wl = {motor_max_speed}\n")
#        elif req_wr > req_wl:
#            req_wl = req_wl / req_wr * motor_max_speed # TODO PUT IN LOG SCALE HERE!!!!!!!!!
#            req_wr = motor_max_speed
            #print(f"we are in wl < wr setting wr = {motor_max_speed}\n")
#        else:
#            req_wl = motor_max_speed
#            req_wr = motor_max_speed
            #print(f"we are in wl = wr setting wl = {motor_max_speed}\n")

#    print(f"motor adjusted to max 55535 required wl: {req_wl:.2f}, required wr: {req_wr:.2f}")

#    return req_wl, req_wr, rotation_angle, check_angle



def set_wheel_speeds(ser, robot, check_angle, target = [10, 0]): # Function runs path planning and drive functions to arrive at tennis ball location
    # inputs: target (array)
    # outputs: distance (float)
    
    ### COMMENTED OUT FOR REAL ROBOT
    # GGB added angle as an output because it didn't like that the function returns three values and we only output 2 (wl and wr)

    if simulation:
        wl, wr, angle, check_angle = calculate_desired_velocities(robot, check_angle, target[0], target[1], max_speed=1, turn_factor = 1)

        return wl, wr
    ###
    if ser.is_open:
        # get wheel rotation speeds & convert to string
        #print("entering cal desired velocities \n\n")
        wl, wr, angle_from_robot_to_target, check_angle = calculate_desired_velocities(robot, check_angle, target[0], target[1], max_speed=1, turn_factor = 1)


        #print(f"wl = {wl} and wr = {wr}\n\n")

        #try:
        #    # changed 100 to 2000 as we are using PWM
        #    wl_str, wr_str = str(int(round(abs(wl)) * 4000)), str(int(round(abs(wr) * 4000)))
        #except:
        #    print("error in the convesion of target velocities into strings")
        
        ccc = 50000
        grad = 800

        # adjusting the values to be good for the input to PWM
        if wl < 0:
            #wl = -(10000 + abs(wl) * ((30000 - 10000) / 25))
            wl = -ccc + grad * wl
            #print(f"scaled wl =  {wl}  \n")
        else:
            #wl = (10000 + abs(wl) * ((30000 - 10000) / 25))
            wl = ccc + grad * wl
            #print(f"scaled wl =  {wl}  \n")
        if wr < 0:
            #wr = -(10000 + abs(wr) * ((30000 - 10000) / 25))
            wr = -ccc + grad * wr
            #print(f"scaled wr =  {wr}  \n")
        else: 
            #wr = (10000 + abs(wr) * ((30000 - 10000) / 25))
            wr = ccc + grad * wr
            #print(f"scaled wr =  {wr}  \n")
        
        time.sleep(0.1)
        
        
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

    return wl, wr, angle_from_robot_to_target, check_angle
