#!/usr/bin/env python3

import wpilib
import wpilib.drive
import wpimath.kinematics
import wpimath.geometry
import wpimath.units
import wpimath.filter
import controls
import swerve_drive_params
import swerve_module

class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        self.robot_params = swerve_drive_params.robot_description()
        
        # Front Left
        self.robot_params._swerve_drives._fl.setup_drive_motor(22)
        self.robot_params._swerve_drives._fl.setup_angle_motor(23)
        self.robot_params._swerve_drives._fl.setup_angle_encoder(32, -13.008)
        self.fl = swerve_module.SwerveModule(self.robot_params._swerve_drives._fl)

        # Rear Left
        self.robot_params._swerve_drives._rl.setup_drive_motor(24)
        self.robot_params._swerve_drives._rl.setup_angle_motor(25)
        self.robot_params._swerve_drives._rl.setup_angle_encoder(33, -322.646)
        self.rl = swerve_module.SwerveModule(self.robot_params._swerve_drives._rl)

        # Front Right
        self.robot_params._swerve_drives._fr.setup_drive_motor(20)
        self.robot_params._swerve_drives._fr.setup_angle_motor(21)
        self.robot_params._swerve_drives._fr.setup_angle_encoder(31, -64.951)
        self.fr = swerve_module.SwerveModule(self.robot_params._swerve_drives._fr)

        # Rear Right
        self.robot_params._swerve_drives._rr.setup_drive_motor(26)
        self.robot_params._swerve_drives._rr.setup_angle_motor(27)
        self.robot_params._swerve_drives._rr.setup_angle_encoder(30, -305.420)
        self.rr = swerve_module.SwerveModule(self.robot_params._swerve_drives._rr)

        #self.launcher = rev.CANSparkMax(7, rev._rev.CANSparkLowLevel.MotorType.kBrushless)

        self.controls = controls.Controls(0)
        self.timer = wpilib.Timer()


    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        self.timer.reset()
        self.timer.start()

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""

    def teleopInit(self):
        """This function is run once each time the robot enters teleop mode."""
        self.timer.reset()
        self.timer.start()

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        #self.launcher.set(self.controls.launcher())

        x = self.controls.forward()
        y = self.controls.horizontal()
        rotate = self.controls.rotate()

        fieldRelative = False
        relativeRotation = wpimath.geometry.Rotation2d()
        #if fieldRelative:
        #    relativeRotation = self.robot._gyro.getRotation2d()

        chassisSpeeds = wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(x, y , rotate, relativeRotation)
        discretized = wpimath.kinematics.ChassisSpeeds.discretize(chassisSpeeds, self.getPeriod())
        fl, fr, rl, rr = self.robot_params._drive_kinematics.toSwerveModuleStates(discretized)

        self.fl.set_swerve_state(fl)
        self.fr.set_swerve_state(fr)
        self.rl.set_swerve_state(rl)
        self.rr.set_swerve_state(rr)

        if self.timer.get() > 0.5:
            print("chassisSpeeds: " + str(chassisSpeeds))
            #print("left forward encoder pos" + str(self.left_forward.encoder.getAbsolutePosition()))
            #print("left rear encoder pos" + str(self.left_rear.encoder.getAbsolutePosition()))
            #print("right forward encoder pos" + str(self.right_forward.encoder.getAbsolutePosition()))
            #print("right rear encoder pos" + str(self.right_rear.encoder.getAbsolutePosition()))

            self.timer.reset()


if __name__ == "__main__":
    wpilib.run(MyRobot)