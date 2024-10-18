import select, sys, machine, time, uselect
import drive, encoder, robot_class, comms

machine.freq(240000000) # set the CPU frequency to 240 MHz

count = 0
global odom_print
odom_print = 1

robot = robot_class.DiffDriveRobot()

i = 0
robot.set_control_mode(1)

#            print("motor ID: ", motor)
while 1:
    
    odom_print = comms.read_comms(robot, odom_print)
    robot.control_loop()
    x, y, th = robot.pose_update()

    if count >= 80:
        if odom_print == 1:
            print(" \nx: ", x, " \ny: ", y, " \nt: ", th)
        count = 0
    
    count = count + 1
    #time.sleep_ms(5)
    #robot.pwm_motor_set(35565, direction = 0, motor = 1) 
    #robot.pwm_motor_set(35565, direction = 0, motor = 0) 
    
    #robot.pwm_motor_set(0, direction = 0, motor = 1) 
    #robot.pwm_motor_set(0, direction = 0, motor = 0) 
    
    #utime.sleep_ms(20)     
