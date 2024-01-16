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
import rev
import sys
import test_imu

class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        self.robot_params = swerve_drive_params.robot_description()
        
        # Front Left
        self.robot_params._swerve_drives._fl.setup_drive_motor(22)
        self.robot_params._swerve_drives._fl.setup_angle_motor(23, pid_p_=0.5)
        self.robot_params._swerve_drives._fl.setup_angle_encoder(32, -0.034180)
        self.fl = swerve_module.SwerveModule(self.robot_params._swerve_drives._fl)
        self.fl.reset_encoders()

        # Rear Left
        self.robot_params._swerve_drives._rl.setup_drive_motor(24)
        self.robot_params._swerve_drives._rl.setup_angle_motor(25, pid_p_=0.5)
        self.robot_params._swerve_drives._rl.setup_angle_encoder(33, -0.098877)
        self.rl = swerve_module.SwerveModule(self.robot_params._swerve_drives._rl)
        self.rl.reset_encoders()

        # Front Right
        self.robot_params._swerve_drives._fr.setup_drive_motor(20)
        self.robot_params._swerve_drives._fr.setup_angle_motor(21, pid_p_=0.5)
        self.robot_params._swerve_drives._fr.setup_angle_encoder(31, 0.241699)
        self.fr = swerve_module.SwerveModule(self.robot_params._swerve_drives._fr)
        self.fr._drive_motor.setInverted(True)
        self.fr.reset_encoders()

        # Rear Right
        self.robot_params._swerve_drives._rr.setup_drive_motor(26)
        self.robot_params._swerve_drives._rr.setup_angle_motor(27, pid_p_=0.5)
        self.robot_params._swerve_drives._rr.setup_angle_encoder(30, -0.388184)
        self.rr = swerve_module.SwerveModule(self.robot_params._swerve_drives._rr)
        self.rr._drive_motor.setInverted(True)
        self.rr.reset_encoders()

        self.gyro = test_imu.TestIMU()
        if "pytest" not in sys.modules:
            self.gyro = wpilib.ADIS16470_IMU()

        self.launcher = rev.CANSparkMax(7, rev._rev.CANSparkLowLevel.MotorType.kBrushless)

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

        # Set the gyro to be 90 degrees off because of the orientation of the roborio.
        # TODO move this into a function.
        angle = self.gyro.getAngle()
        self.gyro.setGyroAngle(angle + 90.0)

        self.timer.reset()
        self.timer.start()

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        self.launcher.set(self.controls.launcher())

        x = self.controls.forward()
        y = self.controls.horizontal()
        rotate = self.controls.rotate()

        fieldRelative = False
        if fieldRelative:
            gyroYaw = self.gyro.getAngle()
            relativeRotation = wpimath.geometry.Rotation2d.fromDegrees(gyroYaw)
            chassisSpeeds = wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(x, y , rotate, relativeRotation)
        else:
            chassisSpeeds = wpimath.kinematics.ChassisSpeeds(x, y, rotate)

        discretized = wpimath.kinematics.ChassisSpeeds.discretize(chassisSpeeds, self.getPeriod())
        fl, fr, rl, rr = self.robot_params._drive_kinematics.toSwerveModuleStates(discretized)

        self.fl.set_swerve_state(fl)
        self.fr.set_swerve_state(fr)
        self.rl.set_swerve_state(rl)
        self.rr.set_swerve_state(rr)

        if self.timer.get() > 0.5:
            #print("left forward encoder pos" + str(self.left_forward.encoder.getAbsolutePosition()))
            #print("left rear encoder pos" + str(self.left_rear.encoder.getAbsolutePosition()))
            #print("right forward encoder pos" + str(self.right_forward.encoder.getAbsolutePosition()))
            #print("right rear encoder pos" + str(self.right_rear.encoder.getAbsolutePosition()))

            self.timer.reset()


if __name__ == "__main__":
    wpilib.run(MyRobot)