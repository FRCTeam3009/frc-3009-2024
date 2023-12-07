'''
    This test module imports tests that come with pyfrc, and can be used
    to test basic functionality of just about any robot.
'''

from pyfrc.tests import *
import swervemotor

def test_swerve_align():
    default_speed = 0.3
    offset = 0.1
    bufferzone = 10

    speed = swervemotor.Motor.align(0, 0)
    assert(0 == speed)

    speed = swervemotor.Motor.align(20, 0)
    assert(default_speed == speed)
    
    speed = swervemotor.Motor.align(bufferzone, 0)
    assert(default_speed == speed)

    speed = swervemotor.Motor.align(offset + .1, 0)
    assert(speed > 0)

    speed = swervemotor.Motor.align(offset, 0)
    assert(0 == speed)

    speed = swervemotor.Motor.align(-offset, 0)
    assert(0 == speed)

    speed = swervemotor.Motor.align(-(offset + .1), 0)
    assert(speed < 0)

    speed = swervemotor.Motor.align(-bufferzone, 0)
    assert(-default_speed == speed)

    speed = swervemotor.Motor.align(-20, 0)
    assert(-default_speed == speed)

    