# Code stack

Produced with assistance from ChatGPT. [reference correctly]

### Hardware stack
**missing connections + power**

| Hardware | Quantity | Purpose | Connections | Power |
|----------|----------|---------|-------------|-------|
|Raspberry Pi 4| 1 | Primary processing unit | GPIO pins - motor control <br> USB port - camera |
|L298N H bridge Driver| 1 | Controls direction and speed of DC motors |
|USB camera | 1 | Provides vision for tennis ball detection |
|Motor| 3 | Drives robot wheels |
|Magnetic encoder| 2 | Provides feedback on motor position and speed |
|Servo| 1 | Controls precise steering motion |

### Software stack
**missing config files + function documentation**

**Languages:** Python, Nix \
**Libraries:** Micropython, CircuitPython, Open Source Computer Vision Library, Numerical Python, Python Standard library\
**Configuration files:** ?\
**Function Documentation:** incomplete

| Function | Purpose | Inputs |Outputs | Example call|
|----------|----------|---------|-------------|-------|
|**Driving**|
|pos_motor| |**pulses** <br> **dir** <br> **motor**|
|pwm_motor| Defines parameters for driving motor |**duty_cycle** (0-65535) <br> **direction** (0 = forwards, 1 = backwards) <br> **motor** selects motor to run (1-3) | ?? |  drive_motor(duty_cycle = duty_cycle, direction = 1, motor = 0) |
|stop_motor| Stops motor driving |
|read_encoder|
|**Detection**|
|is_yellow_or_green| Determines if object is likely a tennis ball based on colour | **r** red (0-255) <br> **g** green (0-255)<br> **b** blue (0-255) | **is_within_range** (True/False) <br> **hue** (?) <br> **saturation** (?) <br> **value** (?) | is_within_range, hue, saturation, value = is_yellow_or_green(r, g, b) |
|**Navigation**|




### Navigation:
**Sensors:** \
**Algorithms:** \
**Integration:**

### Communication:
**Protocols:**\
**Data formats:**\
**Integration:**

### Control systems:
**Algorithms:** \
**Software:** \
**Integration:**

### Motor Control:
**Motor types:** DC motors for movement \
**Control algorithms:** PID (?) for speed + position control \
**Driver integration:** L298N driver controlled via PWM signals \
**Software implementation:** \
**Testing & calibration:**

### Sensor Integration:
**Sensors:** \
**Data handling:** \
**Integration with control systems:** \
**Software implementation:** \
**Testing & calibration:**

### Testing & Calibration:
**Motor & sensing:** \
**Calibration:**

### Documentation & References
**Datasheets:** \
**Guides:** \
**Additional resources:**

### Development & Debugging Tools:
**IDE:** Visual Studio Code \
**Version control:** Git \
**Debugging tools:**

## Code process

#### main
**IF** returning to box because detection failed **OR** full tray \
**foobar()** --> read serial info for robot location \
**calculate distance** --> from robot to box \
**drive_to_target()** --> target is box \
**stop_motor()** --> once arrived at box

**ELSE** still picking up tennis balls \
**stop_motor()** --> to ensure robot starts from rest \
**check distance** --> ensure vision + encoder distance agrees \
**ball_search_drive().init_search** -->  
**drive_to_target()** --> using distance to tennis ball

#### main.foobar()
**read_commands()** --> read serial data from pi pico \
**update_robot()** --> read in x,y,theta for current robot position

#### main.read_commands()
**read serial data** --> reads in single line

#### main.update_robot()
**read x,y,theta** --> reads serial data into robot class variables

#### main.drive_to_target()
**check distance** --> ensures distance to target is within threshold (box or ball) \
**path_planning.set_wheel_speeds()** --> determines wheel speeds to get robot to target location \
**foobar()** --> check robot position \
**recalculate distance** --> to target

#### path_planning.set_wheel_speeds()
**foobar()** --> update robot current position \
**calculate_desired_velocities()** --> calculate velocities to get robot from current position to target \
**convert velocities to strings** --> to pass into PWM command \
**write velocities to PWM** --> passes velocities to drive robot

#### main.stop_motor()
**serial write** --> stop left and right motors

#### path_planning.calculate_desired_velocities()
**calc distance** --> dx and dy from robot to target \
**calc angle** --> rotation required to approach target \
**adjust angle** --> to choose acute equivalent \
**adjust robot speed** --> if close to robot, slow down, else go fast \
**convert to wheel speeds** --> based on desired robot velocity \
**adjust wheel speeds** --> based on max achievable speeds

#### ball_search_drive.init_search()
