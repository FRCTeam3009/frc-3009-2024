'''
    This test module imports tests that come with pyfrc, and can be used
    to test basic functionality of just about any robot.
'''

from pyfrc.tests import *
import math
import wpimath.units
import robot
import constants

def float_equals(a, b):
    if abs(a - b) < 0.00001:
        return True
    return False

assert(float_equals(robot.convert_servo_angle_to_value(0), 0.9))
assert(float_equals(robot.convert_servo_angle_to_value(85), 0.2))
assert(float_equals(robot.convert_servo_angle_to_value(45), 0.5294117647058824))
assert(float_equals(robot.convert_servo_angle_to_value(75), 0.2823529411764706))

assert(float_equals(robot.convert_shooter_angle_to_servo_value(46), 0.9))
assert(float_equals(robot.convert_shooter_angle_to_servo_value(50), 0.7727272727272727))
assert(float_equals(robot.convert_shooter_angle_to_servo_value(68), 0.2))

height = wpimath.units.inchesToMeters(constants.speakerHeight)
radians = math.atan2(height, constants.maxDistanceSpeakerShot)
degrees = wpimath.units.radiansToDegrees(radians)
assert(float_equals(degrees, 22.044729020889886))
radians = math.atan2(height, constants.minDistanceSpeakerShot)
degrees = wpimath.units.radiansToDegrees(radians)
assert(float_equals(degrees, 44.0062344834231))