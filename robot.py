#!/usr/bin/env python3
"""
    This is a good foundation to build your robot code on
"""

import wpilib
import wpilib.drive
import rev
import joystick

class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        self.left_forward_drive_motor = rev.CANSparkMax(20, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.left_forward_steer_motor = rev.CANSparkMax(21, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.left_rear_drive_motor = rev.CANSparkMax(22, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.left_rear_steer_motor = rev.CANSparkMax(23, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.right_forward_drive_motor = rev.CANSparkMax(24, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.right_forward_steer_motor = rev.CANSparkMax(25, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.right_rear_drive_motor = rev.CANSparkMax(26, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.right_rear_steer_motor = rev.CANSparkMax(27, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        
        self.drive = wpilib.drive.MecanumDrive(self.left_forward_drive_motor, self.left_rear_drive_motor, self.right_forward_drive_motor, self.right_rear_drive_motor)
        self.stick = joystick.Joystick(0)
        self.timer = wpilib.Timer()

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        self.timer.reset()
        self.timer.start()

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        self.left_forward_steer_motor.set(0)
        self.left_rear_steer_motor.set(0)
        self.right_forward_steer_motor.set(0)
        self.right_rear_steer_motor.set(0)

        # Drive for two seconds
        if self.timer.get() < 2.0:
            self.drive.driveCartesian(-0.5, 0, 0)  # Drive forwards at half speed
        else:
            self.drive.driveCartesian(0, 0, 0)  # Stop robot

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        self.drive.driveCartesian(self.stick.forward(), self.stick.horizontal(), self.stick.rotate())
        self.left_forward_steer_motor.set(0)
        self.left_rear_steer_motor.set(0)
        self.right_forward_steer_motor.set(0)
        self.right_rear_steer_motor.set(0)

if __name__ == "__main__":
    wpilib.run(MyRobot)