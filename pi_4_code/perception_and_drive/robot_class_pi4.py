import numpy as np
import time

class DiffDriveRobot:
    
    # duty_cycle - range 0-65535
    # Direction - 0 = forwards, 1 = backwards
    # Motor - selects which motor to run

    def __init__(self, inertia=5, dt=0.1, drag=0.2, wheel_radius=0.05, wheel_sep=0.15):

        
        # define inital positions / rotations 
        self.x = 0 # y-position
        self.y = 0 # y-position 
        self.th = 0 # orientation

        self.r = wheel_radius
        self.l = wheel_sep

        self.wr = 0
        self.wl = 0

        self.stop_str_1 = "PWM 0 0 0  \n\r"
        self.stop_str_2 = "PWM 1 1 0  \n\r"

        self.right_wheel_gain_factor = 1 #0.88 
        self.left_wheel_gain_factor = 1 #0.97

    def drive_wheel(self, ser, speed, direction = "forwards", wheel = "left"):

        if wheel == "left": 
            wheel = 0
            speed = speed * self.left_wheel_gain_factor
        elif wheel == "right": 
            wheel = 1
            speed = speed * self.right_wheel_gain_factor

        if direction == "forwards": 
            direction = 0
        if direction == "backwards": 
            direction = 1
        # set min and max values for the speed and direction

        speed = max(0, min(55535, speed))
        #print(f"speed is = {speed}\n\n")
        direction = max(0, min(1, direction))
        motor_command = "PWM " + str(direction) + " " + str(wheel) + " " + str(speed) +"\n"
        #print(motor_command)
        time.sleep(0.1)
        ser.write(motor_command.encode())

    def servo_close(self, ser):
        servo_str = "SER 1 \n"
        ser.write(servo_str.encode())
        time.sleep(0.2)
        ser.write(servo_str.encode())


    def servo_open(self, ser):
        servo_str = "SER 0 \n"
        ser.write(servo_str.encode())
        time.sleep(0.2)
        ser.write(servo_str.encode())

    def stop_motors(self, ser):
        # function that writes the stop command to both drive motors to stop the robot
        ser.write(self.stop_str_1.encode())
        #time.sleep(0.1)
        ser.write(self.stop_str_2.encode())
        ser.write(self.stop_str_2.encode())
        #time.sleep(1)

    def update_pose(self, x, y, theta):
        # updates the pose of the robot

        self.x = x
        self.y = y
        self.th = theta 
        return 1 

    def update_y(self, y):
        self.y = y 

    def update_x(self, x):
        self.x = x

    def update_theta(self, theta):
        self.th = theta 
