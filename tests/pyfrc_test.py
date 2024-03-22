'''
    This test module imports tests that come with pyfrc, and can be used
    to test basic functionality of just about any robot.
'''

from pyfrc.tests import *
import robot

def float_equals(a, b):
    if abs(a - b) < 0.00001:
        return True
    return False

assert(float_equals(robot.convert_servo_angle_to_value(0), 0.9))
assert(float_equals(robot.convert_servo_angle_to_value(85), 0.2))
assert(float_equals(robot.convert_servo_angle_to_value(45), 0.5294117647058824))
assert(float_equals(robot.convert_servo_angle_to_value(75), 0.2823529411764706))

# TODO fix this
#assert(float_equals(robot.convert_shooter_angle_to_servo_value(44), 0))
#assert(float_equals(robot.convert_shooter_angle_to_servo_value(22), 1))
