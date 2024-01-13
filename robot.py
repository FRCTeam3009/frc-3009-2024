#!/usr/bin/env python3

import wpilib
import wpilib.drive
import controls
import swerve_drive_params
import rev
class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        robot = swerve_drive_params.robot_description()
        
        # Front Left
        robot._swerve_drives._fl.setup_drive_motor(22)
        robot._swerve_drives._fl.setup_angle_motor(23)
        robot._swerve_drives._fl.setup_angle_encoder(32, -13.008)

        # Rear Left
        robot._swerve_drives._rl.setup_drive_motor(24)
        robot._swerve_drives._rl.setup_angle_motor(25)
        robot._swerve_drives._rl.setup_angle_encoder(33, -322.646)

        # Front Right
        robot._swerve_drives._fr.setup_drive_motor(20)
        robot._swerve_drives._fr.setup_angle_motor(21)
        robot._swerve_drives._fr.setup_angle_encoder(31, -64.951)

        # Rear Right
        robot._swerve_drives._rr.setup_drive_motor(26)
        robot._swerve_drives._rr.setup_angle_motor(27)
        robot._swerve_drives._rr.setup_angle_encoder(30, -305.420)

        self.launcher_test = rev.CANSparkMax(7, rev._rev.CANSparkLowLevel.MotorType.kBrushless)

        self.controls = controls.Controls(0)
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
        self.left_forward.drive(self.controls.forward())
        self.left_rear.drive(self.controls.forward())
        self.right_forward.drive(self.controls.forward())
        self.right_rear.drive(self.controls.forward())

        self.launcher_test.set(self.controls.launcher())

        if self.controls.align_zero():
            self.left_forward.align_zero()
            self.left_rear.align_zero()
            self.right_forward.align_zero()
            self.right_rear.align_zero()
        else:
            self.left_forward.steer(self.controls.rotate())
            self.left_rear.steer(self.controls.rotate())
            self.right_forward.steer(self.controls.rotate())
            self.right_rear.steer(self.controls.rotate())
        

        if self.timer.get() > 0.5:
            #print("left forward encoder pos" + str(self.left_forward.encoder.getAbsolutePosition()))
            #print("left rear encoder pos" + str(self.left_rear.encoder.getAbsolutePosition()))
            #print("right forward encoder pos" + str(self.right_forward.encoder.getAbsolutePosition()))
            #print("right rear encoder pos" + str(self.right_rear.encoder.getAbsolutePosition()))

            self.timer.reset()


if __name__ == "__main__":
    wpilib.run(MyRobot)