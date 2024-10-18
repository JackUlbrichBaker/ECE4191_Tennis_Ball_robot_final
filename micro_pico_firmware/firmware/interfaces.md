# Motor driver interface info
Controlling the motor driver is as easy as sending USB serial data to the pi pico and reading serial data.


Motor 0 = LEFT
Motor 1 = RIGHT 
Direction 0 = Forwards
Direction 1 = Backwards 

## Commands:

`VEL 0-1 0-1 0-254`

`PWM 0-1 0-1 0-65535`

`POS 0-1 0-1 0-254`

`GVL`

`ENC -`

`SEN -`

`CON -`

####  PWM - Open Loop Velocity

This sends an open loop PWM duty cycle to the motors. Range:  0 - 65535.

units: duty cycle % 0 - 65535

command syntax:
**PWM** [motor ID] [direction] [duty_cycle]

####  VEL - Closed Loop Velocity

Receives an angular velocity to drive the wheels to.  Needs the wheel diameter and encoder data to be configured.
Angular_velocity commanded will be divided by 100 then sent to wheels - i.e [1000] will be 10 rad/s

units: rad/sec

command syntax:
**VEL** [motor ID] [direction] [angular_velocity]

#### POS - Position

This command receives a motor ID (1 or 0) and a position to drive the motor to.  It will drive the motors until the encoders read the specified value.
eg: POS 10 will drive the motors until the encoders increase by 10 pulses.
Note that the gearbox ratio + CPR of the motor encoders will affect how many physical rotations this is (rads) 

units: enc pulses

command syntax:
**POS** [motor ID] [direction] [encoder_pulses]

#### GVL - Get Velocity

returns the current angular velocity of the wheels.

command syntax:
**GVL** 

#### ENC - Encoder

This will command the motor driver to send the current encoder values via serial

command syntax:
**ENC**

#### SEN - Sensor 

This will send the current sensor data via serial

#### CON - Configuration

This will dump the motor drivers config to serial
