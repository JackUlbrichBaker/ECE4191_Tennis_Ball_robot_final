from machine import Pin, PWM, Timer
import utime
import encoder

m1 = Pin(1, Pin.OUT)
m2 = Pin(2, Pin.OUT)
m3 = Pin(3, Pin.OUT)
m4 = Pin(4, Pin.OUT)
 
enable1 = PWM(Pin(0), freq=20000, duty_u16=0)
enable2 = PWM(Pin(5), freq=20000, duty_u16=0)
#enable1 = Pin(0, Pin.OUT)
#enable2 = Pin(6, Pin.OUT)
print(enable2)
def pwm_motor(duty_cycle, direction, motor):
    # Inputs:
    # duty_cycle - range 0-65535
    # Direction - 0 = forwards, 1 = backwards
    # Motor - selects which motor to run
    
    #enable1.duty_u16(0)  #disable the h bridge to prevent shorts
    #enable2.duty_u16(0)
    motor = int(motor)
    #print("motor ID:", motor)
    utime.sleep_ms(2)

    if motor == 0:
        enable2.duty_u16(duty_cycle)
        if direction == 1:
            m3.off()
            m4.on()
        if direction == 0:
            m3.on()
            m4.off()
    elif motor == 1:
        enable1.duty_u16(duty_cycle)
        if direction == 1:
            m1.off()
            m2.on()
        if direction == 0:
            m1.on()
            m2.off()

def pos_motor(pulses, direction, motor):
    print("NOT IMPLEMENTED")
#       TODO 
    #    encoder_current = encoder.read_encoder()
#    if direction == 0:
#        direction = -1
#    else:
#        direction = 1
#    goal_pulses = pulses + encoder_pulses*direction
#    while encoder_current != goal_pulses:
#        print(encoder_current)
#        pwm_motor(65535, dir, motor)
#        encoder_pulses = encoder.value()
    
def vel_motor(velocity, dir, motor, recur_count = 0, integrated_err = 0):
    control_loop_period = 20
    dt = 5/control_loop_period # 50ms between control cycles
    curr_vel_l, curr_vel_r = encoder.read_velocity()
    if dir == 0:
        curr_vel = curr_vel_l
    else:
        curr_vel = curr_vel_r
    err = velocity - curr_vel
    integrated_err = integrated_err + dt * err
    if integrated_err > 1000:
        integrated_err = 1000          # set upper limit to avoid integrator windup
    Ki = 50000
    Kp = 55000
    duty_cycle = err*Kp + integrated_err*Ki
    if duty_cycle < 0:
        duty_cycle = 0
    #print("dut_cy: ", duty_cycle, "error: ", err,"int err: ", integrated_err)
    pwm_motor(int(duty_cycle), dir, motor)
    recur_count += 1
    if recur_count == 10:
        #stop_motor()
        return 1
    utime.sleep_ms(control_loop_period)
    vel_motor(velocity, dir, motor, recur_count, integrated_err)


def stop_motor():
    enable1.duty_u16(0)
    enable2.duty_u16(0)
