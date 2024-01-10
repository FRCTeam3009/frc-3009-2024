#!/usr/bin/env python3
"""
    This is a good foundation to build your robot code on
"""

import wpilib
import wpilib.drive
import joystick
import swervemotor
import rev
class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        self.left_forward = swervemotor.Motor(22, 23, 32, -13.008, True)
        self.left_rear = swervemotor.Motor(24, 25, 33, -322.646, True)
        self.right_forward = swervemotor.Motor(20, 21, 31, -64.951)
        self.right_rear = swervemotor.Motor(26, 27, 30, -305.420)

        self.launcher_test = rev.CANSparkMax(7, rev._rev.CANSparkLowLevel.MotorType.kBrushless)


        self.left_forward.align_zero()
        self.left_rear.align_zero()
        self.right_forward.align_zero()
        self.right_rear.align_zero()

        self.joystick = joystick.Joystick(0)
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

    def teleopInit(self):
        """This function is run once each time the robot enters teleop mode."""
        self.timer.reset()
        self.timer.start()

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        self.left_forward.drive(self.joystick.forward())
        self.left_rear.drive(self.joystick.forward())
        self.right_forward.drive(self.joystick.forward())
        self.right_rear.drive(self.joystick.forward())

        self.launcher_test.set(self.joystick.launcher())

        if self.joystick.align_zero():
            self.left_forward.align_zero()
            self.left_rear.align_zero()
            self.right_forward.align_zero()
            self.right_rear.align_zero()
        else:
            self.left_forward.steer(self.joystick.rotate())
            self.left_rear.steer(self.joystick.rotate())
            self.right_forward.steer(self.joystick.rotate())
            self.right_rear.steer(self.joystick.rotate())
        

        if self.timer.get() > 0.5:
            #print("left forward encoder pos" + str(self.left_forward.encoder.getAbsolutePosition()))
            #print("left rear encoder pos" + str(self.left_rear.encoder.getAbsolutePosition()))
            #print("right forward encoder pos" + str(self.right_forward.encoder.getAbsolutePosition()))
            #print("right rear encoder pos" + str(self.right_rear.encoder.getAbsolutePosition()))

            self.timer.reset()


if __name__ == "__main__":
    wpilib.run(MyRobot)