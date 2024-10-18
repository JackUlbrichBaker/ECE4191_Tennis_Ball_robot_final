import sys, os, time, select, machine

sensor_temp = machine.ADC(4)
conversion_factor = 3.3 / (65535)


def read_comms(robot):
    if select.select([sys.stdin],[],[],0)[0]:
        command = sys.stdin.readline()
        com = command[0:3]
        if com == "TEM": 
            reading = sensor_temp.read_u16() * conversion_factor
            temperature = 27 - (reading - 0.706)/0.001721
            print("Temp: " + str(temperature))
        
        elif com == "ENC":
            #encoder.read_encoder()
            enc_l, enc_r = robot.get_wheel_encoders()
            print(f"encoder values:  right: {enc_r}, left: {enc_l}")
            
        elif com == "RES":
            robot.set_pos(0,0,0)
            
        elif com == "SER":   ## SER 0  closes servo
            on = int(command[4])
            if on == 1:
                robot.servo_close()
            elif on == 0:
                robot.servo_open()
            else:
                return

        elif com == "ROL":   ## ROL 0  turns rollers off etc
            on = int(command[4])
            robot.rollers(on)

        elif com == "GVL":
            #encoder.read_velocity()
            print("TODO:  Implement get velocity")

        elif com == "POS":
            motor = int(command[4])
            dir = int(command[6])
            pulses = int(command[8:11])
            #drive.pos_motor(pulses, dir, motor)
            robot.set_control_mode(3)
            robot.pos_control(motor, dir, pulses)

        elif com == "VEL":
            motor = int(command[4])
            dir = int(command[6])
            vel = float(command[8:13])/100
            #print("!!!!!!!!!velocity commanded: ",vel)
            if vel > 10:
                vel = 10
            robot.set_control_mode(2)
            robot.vel_control(motor, dir, vel)

        elif com == "PWM":
            motor = int(command[4])
            dir = int(command[6])
            duty_cycle = int(command[8:13])
            if duty_cycle > 65535:
                duty_cycle = 65535
#            print("motor ID: ", motor)
#            print("Direction: ", dir)
#            print("duty_cycle:", duty_cycle)
            robot.set_control_mode(1)
            robot.vel_control(motor, dir, duty_cycle)
