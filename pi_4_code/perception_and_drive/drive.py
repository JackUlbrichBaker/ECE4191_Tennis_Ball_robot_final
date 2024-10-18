#  THIS FILE ONLY ACCEPTS WORLD COORDS + ROBOT COORDS.
#
import numpy as np
import time


def dist_to_ball(robot, target):
    target_x  = target(0)
    target_y = target(1)
    distance = np.sprt((target_y - robot.y)**2 + (target_x - robot.x)**2)
    return distance

def angle_to_ball(robot, target):
    target_x  = target(0)
    target_y = target(1)
    dy = (target_y - robot.y)
    dx = (target_x - robot.x)
    angle = np.arctan2(dy, dx)
    return angle

def pivot_to_angle(robot, angle, threshold = np.deg2rad(1)):

    # THIS IS BLOCKING
    # Takes a robot and angle, then pivots the robot to face the target angle.
    # Please note this angle is relative to the ROBOT
    # default tolerance for angle is 1 degree

    if angle > 2 * np.pi:
        angle -= 2*np.pi 
    elif angle < 0:
        angle -= 2*np.pi

    angle_to_ball = angle_to_ball(robot, target)

    while (angle_to_ball - angle) > threshold:

        angle_to_ball = angle_to_ball(robot, angle)

        speed = 65535
        robot.drive_wheel(speed, wheel = "left")
        robot.drive_wheel(speed, wheel = "right")

        time.sleep(1)
        speed = 65535
        robot.drive_wheel(speed, wheel = "left")
        robot.drive_wheel(speed, wheel = "right")

        #TODO

