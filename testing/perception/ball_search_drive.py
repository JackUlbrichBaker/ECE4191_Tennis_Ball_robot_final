import cv2
import numpy as np
import serial

from ball_detect import detect_ball
# TODO: from robot import drive


ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
ser.reset_input_buffer()
reset_str = str("RES")
stop_str_1 = "PWM 0 0 0  \n\r "
stop_str_2 = "PWM 1 0 0\n\r"        
ser.write(reset_str.encode())

def read_commands(ser):         # function to read from the USB serial stream to get data from pi pico
    # inputs: ser ( serial object )
    # outputs: line ( string )

    #while ser.in_waiting > 0:
    line = ser.readline().decode('utf-8').rstrip()
    time.sleep(0.01)
    print("input string: ", line)
    return line

def update_robot(ser_data):      # this accepts the data from the pi pico in the form of: "\nx: 0.12343 \ny: 1.2345 \nt: 90" 
    # inputs:  ser_data (string) 

    #ser_data = pd.to_numeric(ser_data)
    if ser_data is None: 
        print("no valid data")
        return
    coord = ser_data[0:2]       # this can be x, y or t (theta) 
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


def pivot_to_search(angle,ser):
    # Function makes robot pivot on the spot using a set angle
    d_wheel_rotate = angle * 14
    
    if angle > 0:
        string = "PWM 0 0 " + str(d_wheel_rotate) + "\n"
        ser.write(string.encode())
        string = "PWM 1 1 " + str(d_wheel_rotate) + "\n"
        ser.write(string.encode())

    if angle <= 0:
        string = "PWM 0 1 " + str(d_wheel_rotate) + "\n"
        ser.write(string.encode())
        string = "PWM 1 0 " + str(d_wheel_rotate) + "\n"
        ser.write(string.encode())
    print(f"Pivoting at angle: {angle}")
    return

def expand_search():
    print("Driving forwards to find more balls")
    # TODO: drive around to look for ball - not done yet
    

    ser.write(string.encode())
    # might also be worth having a stop and pivot condition for if we've gone too far off the edge of the tennis court?
    return

def calculate_distance(real_radius, image_radius, focal_length):
    # Calculate the distance using the formula
    distance = (real_radius * focal_length) / image_radius
    return distance

def get_ball_distance(detected_circle):
    f_x = 656.7164179104477  # Camera intrinsic parameter: Focal length in x direction (pixels)
    R_real = 0.0335  # Real-world radius of ball iin meters

    x, y, r =  detected_circle
    # Calculate distance to the ball
    distance = calculate_distance(R_real, r, f_x)

    print(f"The distance to the ball is: {distance}")

    return distance

def drive_to_distance(distance):
    print(f"Driving {distance} meters...")
    # TODO: drive a specified distance
    return

def main():
    final_circle = None
    #angles = [0, 45, 90, 135, 180, 0 -45, -90, -135, -180]
    angles = [0,45,0,-45]
    ball_found = False
    
    # Repeat a ball search sequence until a ball can be found
    while ball_found == False:

        # Search for ball
        angle_idx = 0

        while final_circle is None:        
            pivot_to_search(angles[angle_idx],ser)
            final_circle = detect_ball(view_mode = False)

            angle_idx+=1

            # If no ball is detected in robot's current position
            if angle_idx > len(angles)-1:
                print("No ball detected in area. Driving elsewhere...")
                # Drive elsewhere
                expand_search()
                break
        
        ball_found = True
    
    # Ball is found, measure the distance
    ball_dist = get_ball_distance(final_circle)

    # Drive to that distance
    drive_to_distance(ball_dist)

    return 



if __name__ == "__main__":
    main()
