#!/usr/bin/env python3
"""
    This is a good foundation to build your robot code on
"""

import wpilib
import wpilib.drive
import joystick
import swervemotor
class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        self.left_forward = swervemotor.Motor(22, 23, 32)
        self.left_rear = swervemotor.Motor(24, 25, 33)
        self.right_forward = swervemotor.Motor(20, 21, 30)
        self.right_rear = swervemotor.Motor(26, 27, 31)

        self.stick = joystick.Joystick(0)
        self.timer = wpilib.Timer()



    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        self.timer.reset()
        self.timer.start()

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        self.left_forward.setzero()
        self.left_rear.setzero()
        self.right_forward.setzero()
        self.right_rear.setzero()

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        self.left_forward.drive(self.joystick.forward())
        self.left_rear.drive(self.joystick.forward())
        self.right_forward.drive(self.joystick.forward())
        self.right_rear.drive(self.joystick.forward())

        self.left_forward.steer(self.joystick.horizontal())
        self.left_rear.steer(self.joystick.horizontal())
        self.right_forward.steer(self.joystick.horizontal())
        self.right_rear.steer(self.joystick.horizontal())

        print("left rear encoder" + str(self.left_rear_encoder_motor.getPosition()))

if __name__ == "__main__":
    wpilib.run(MyRobot)