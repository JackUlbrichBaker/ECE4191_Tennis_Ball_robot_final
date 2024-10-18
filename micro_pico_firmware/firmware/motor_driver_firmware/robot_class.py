import math, time
from machine import Pin, PWM, Timer
import encoder

class DiffDriveRobot:
    
    def __init__(self,inertia=5, dt=0.1, drag=0.2, wheel_radius=2.65, wheel_sep=28):
        # Wheel radius + wheel separation are in CM!!!!!!!!!!!


        # define inital positions / rotations 
        self.x = 0.0 # y-position
        self.y = 0.0 # y-position 
        self.th = 0 # orientation
        
        self.wl = 0.0 #rotational velocity left wheel
        self.wr = 0.0 #rotational velocity right wheel
        
        self.enc_r_old = 0  # historic Encoder positions of each wheel
        self.enc_l_old = 0

        self.enc_r = 0  # Encoder positions of each wheel
        self.enc_l = 0

        self.CPR = 12
        self.gearbox_ratio = 75

        self.I = inertia
        self.d = drag
        self.dt = dt
        
        self.r = wheel_radius
        self.l = wheel_sep

        self.drive_mode_list = ["STOP", "PWM", "VEL", "POS"]
        self.drive_mode = self.drive_mode_list[0]

        self.PWM_left = 0
        self.PWM_right = 0

        self.goal_pulses_left = 0
        self.goal_pulses_right = 0

        self.goal_velocity_left = 0
        self.goal_velocity_right = 0


        self.m1 = Pin(2, Pin.OUT)
        self.m2 = Pin(3, Pin.OUT)
        self.m3 = Pin(4, Pin.OUT)
        self.m4 = Pin(5, Pin.OUT)

        self.roller_pin = Pin(6, Pin.OUT)
        self.zero_servo_pos_pwm = 1802
        self.max_servo_pos_pwm = 7864
        self.servo_pin = PWM(Pin(8, freq=50, duty_u16=self.max_servo_pos_pwm))
        
        self.enable1 = PWM(Pin(0), freq=20000, duty_u16=0)
        self.enable2 = PWM(Pin(6), freq=20000, duty_u16=0)

        self.ticks_ms = time.ticks_ms()
        self.ticks_delta  = 0


        self.Ki = 50000
        self.Kp = 55000
        
        self.err_left = 0
        self.err_right = 0

        self.integrated_err_left = 0
        self.integrated_err_right = 0

    def get_wheel_encoders(self):
        return self.enc_l, self.enc_r

    def update_wheel_encoders(self, enc_r, enc_l):
        self.enc_l = self.enc_l + enc_l
        self.enc_r = self.enc_r + enc_r

    def set_control_mode(self, mode):
        if mode >= len(self.drive_mode_list) or mode < 0:
            print("invalid mode!!!!!!")
            return
        self.drive_mode = self.drive_mode_list[mode]
        print(f"set mode to {self.drive_mode}")

    def vel_control_set(self, motor, dir,  vel):
        if motor == 0:
             if dir == 0:
                 self.goal_velocity_left =  vel
             else:                         
                 self.goal_velocity_left =  vel
        elif motor == 1:                  
             if dir == 0:                
                 self.goal_velocity_right = vel
             else:                      
                 self.goal_velocity_right = vel


    def pos_control_set(self, motor, dir,  pulses):
        if motor == 0:
             if dir == 0:
                 self.goal_pulses_left = self.enc_l + pulses
             else:                                          
                 self.goal_pulses_left = self.enc_l - pulses
        elif motor == 1:                                    
             if dir == 0:                                   
                 self.goal_pulses_right = self.enc_r + pulses
             else:                                          
                 self.goal_pulses_right = self.enc_r - pulses


    def base_position(self):
        new_enc_l, new_enc_r = encoder.read_encoder()
        d_enc_r = self.enc_r -  new_enc_r
        d_enc_l = self.enc_l -  new_enc_l
        
        if d_enc_r > 1000:
            d_enc_r = 0
        if d_enc_l > 1000:
            d_enc_l = 0

    # Calculate the change in the wheels rotation
        d_theta_l = d_enc_l/(self.CPR*self.gearbox_ratio)
        d_theta_r = d_enc_r/(self.CPR*self.gearbox_ratio)
        
        
        d_wheel_r = d_theta_r*2*math.pi*self.r
        d_wheel_l = d_theta_l*2*math.pi*self.r
        #print("wheel disp", d_wheel_r)
    # Calculate the change in the forward direction and rotationally
        d_disp = (d_wheel_r + d_wheel_l)/2.0       	# change in forward displacement
        d_theta = (d_wheel_l - d_wheel_r)/(self.l)    	# change in rotation
        
        self.enc_r = new_enc_r
        self.enc_l = new_enc_l

        return d_disp, d_theta

    def get_pos(self):
        return self.x, self.y, self.th
    
    def set_pos(self,x,y,theta):
        self.x = x
        self.y = y
        self.th = theta
        return 1
    
    def get_control_mode(self):
        return self.drive_mode

    def control_loop(self):
        if self.drive_mode == "STOP": 
            self.stop_motor()
       # elif self.drive_mode == "PWM": 
       #     self.pwm_motor_set(self.PWM_left, 0, 1)
        elif self.drive_mode == "VEL": 
            self.vel_motor_control_loop()
        elif self.drive_mode == "POS": 
            self.pos_motor_control_loop()

    # Kinematic motion model
    def pose_update(self):
        d_disp, d_theta = self.base_position() 
    
        self.x = self.x + d_disp*math.cos(self.th + d_theta/2)             # updates x position in world frame 
        self.y = self.y + d_disp*math.sin(self.th + d_theta/2)             # updates y position in world frame 
        self.th = self.th + d_theta                            # updates th position in world frame 
        
        # checks if theta is over 2pi or less then 0, then converts back to being between 0 and 2pi 
        if self.th < 0:
            self.th = self.th + 2*math.pi
        elif self.th > 2*math.pi:
            self.th = self.th - 2*math.pi
        #print("\nx: ", self.x, "\ny: ", self.y, "\nt: ", self.th)
        return self.x, self.y, self.th

    def pwm_motor_set(self, duty_cycle, direction, motor):

         # Inputs:
         # duty_cycle - range 0-65535
         # Direction - 0 = forwards, 1 = backwards
         # Motor - selects which motor to run
         
         #enable1.duty_u16(0)  #disable the h bridge to prevent shorts
         #enable2.duty_u16(0)
         motor = int(motor)
         duty_cycle = min(65535, duty_cycle)

         #print("motor ID:", motor)
         self.PWM_left = duty_cycle

         if motor == 0:
             self.enable2.duty_u16(duty_cycle)
             if direction == 1:
                 self.m3.off()
                 self.m4.on()
             if direction == 0:
                 self.m3.on()
                 self.m4.off()
         elif motor == 1:
             self.enable1.duty_u16(duty_cycle)
             if direction == 1:
                 self.m1.off()
                 self.m2.on()
             if direction == 0:
                 self.m1.on()
                 self.m2.off()

        
    def time_calc(self):
        loop_time = time.ticks_diff(self.ticks_ms, time.ticks_ms())
        self.ticks_ms = time.ticks_ms()
        self.ticks_diff = loop_time

    def pos_motor_control_loop(self):
        # TODO implement PROPER closed loop position control
        
        if self.goal_pulses_left > self.enc_l:
            self.pwm_motor_set(65535, 0, 0)
        elif self.goal_pulses_left < self.enc_l:
            self.pwm_motor_set(65535, 0, 0)
        else:
            self.pwm_motor_set(0, 0, 0)

        if self.goal_pulses_right > self.enc_r:
            self.pwm_motor_set(65535, 0, 1)
        elif self.goal_pulses_right < self.enc_r:
            self.pwm_motor_set(65535, 1, 1)
        else:
            self.pwm_motor_set(0, 0, 1)

    def vel_motor_control_loop(self):
        self.time_calc()

        dt = self.ticks_diff    # difference in time since last control loop
        
        curr_vel_l = self.wl
        curr_vel_r = self.wr


        self.err_left = self.goal_velocity_left - curr_vel_l
        self.err_right = self.goal_velocity_right - curr_vel_r

        self.integrated_err_left = self.integrated_err_left + dt * self.err_left
        self.integrated_err_right = self.integrated_err_right + dt * self.err_right

       # if self.integrated_err_left > 10000:
       #     self.integrated_err_left = 10000          # set upper limit to avoid integrator windup

       # if self.integrated_err_right > 10000:
       #     self.integrated_err_right = 10000          # set upper limit to avoid integrator windup
        
        # set upper limit of 10,000 to avoid integrator windup
        # TODO test integrator windup value

        self.integrated_err_left = min(10000, self.integrated_err_left)
        self.integrated_err_right = min(10000, self.integrated_err_right)

        self.Ki = 50000
        self.Kp = 55000

        duty_cycle_left = self.err_left * self.Kp + integrated_err_left * self.Ki
        duty_cycle_right = self.err_right * self.Kp + integrated_err_right * `self.Ki

       # if duty_cycle_left < 0:
       #     duty_cycle_left = 0

       # if duty_cycle_right < 0:
       #     duty_cycle_right = 0

       # set duty_cycle to a minimum of 0
        duty_cycle_left = max(duty_cycle_left, 0)
        duty_cycle_right = max(duty_cycle_right, 0)


        #print("dut_cy: ", duty_cycle, "error: ", err,"int err: ", integrated_err)
        self.pwm_motor_set(int(duty_cycle_left), dir, 0)
        self.pwm_motor_set(int(duty_cycle_right), dir, 1)

    def stop_motor(self):
        self.pwm_motor_set(0, 0, 0)
        self.pwm_motor_set(0, 0, 1)

    def servo_close(self):
        self.servo_pin.duty_u16(self.zero_servo_pos_pwm)

    def servo_open(self):
        self.servo_pin.duty_u16(self.max_servo_pos_pwm)

    def rollers(self, on):
        if type(on) != bool:
            print("invalid type for roller command")
            return
        self.roller_pin.value(on)
