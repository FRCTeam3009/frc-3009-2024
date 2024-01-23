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
import motorParams
import encoderParams
from photonlibpy import photonCamera,photonTrackedTarget
import ntcore

class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        nt = ntcore.NetworkTableInstance.getDefault()
        nt.startServer()
        sd = nt.getTable("SmartDashboard")

        # Front Left
        fldriveMotorParams = motorParams.Motorparams(22)
        flangleMotorParams = motorParams.Motorparams(23, 0.5)
        flEncoderParams = encoderParams.EncoderParams(32, -0.105713)
        flParams = swerve_drive_params.SwerveDriveParams(fldriveMotorParams, flangleMotorParams, flEncoderParams)

        # Rear Left
        rldriveMotorParams = motorParams.Motorparams(24)
        rlangleMotorParams = motorParams.Motorparams(25, 0.5)
        rlEncoderParams = encoderParams.EncoderParams(33, -0.098877)
        rlParams = swerve_drive_params.SwerveDriveParams(rldriveMotorParams, rlangleMotorParams, rlEncoderParams)

        # Front Right
        frdriveMotorParams = motorParams.Motorparams(20)
        frangleMotorParams = motorParams.Motorparams(21, 0.5)
        frEncoderParams = encoderParams.EncoderParams(31, 0.217529)
        frParams = swerve_drive_params.SwerveDriveParams(frdriveMotorParams, frangleMotorParams, frEncoderParams)
        
        # Rear Right
        rrdriveMotorParams = motorParams.Motorparams(26)
        rrangleMotorParams = motorParams.Motorparams(27, 0.5)
        rrEncoderParams = encoderParams.EncoderParams(30, -0.388184)
        rrParams = swerve_drive_params.SwerveDriveParams(rrdriveMotorParams, rrangleMotorParams, rrEncoderParams)

        self.robot_params = swerve_drive_params.robot_description(flParams, frParams, rlParams, rrParams)

        self.fl = swerve_module.SwerveModule(self.robot_params.fl)
        self.fl.reset_encoders()
        self.rl = swerve_module.SwerveModule(self.robot_params.rl)
        self.rl.reset_encoders()
        self.fr = swerve_module.SwerveModule(self.robot_params.fr)
        self.fr._drive_motor.setInverted(True)
        self.fr.reset_encoders()
        self.rr = swerve_module.SwerveModule(self.robot_params.rr)
        self.rr._drive_motor.setInverted(True)
        self.rr.reset_encoders()

        self.gyro = test_imu.TestIMU()
        if "pytest" not in sys.modules:
            self.gyro = wpilib.ADIS16470_IMU()

        self.launcher = rev.CANSparkMax(7, rev._rev.CANSparkLowLevel.MotorType.kBrushless)
        self.launcher2 = rev.CANSparkMax(8, rev._rev.CANSparkLowLevel.MotorType.kBrushless)
        self.launcher2.follow(self.launcher, True)

        self.limelight1 = photonCamera.PhotonCamera("limelight1")

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

        cameraResult1 = self.limelight1.getLatestResult()

        targetFound = False
        targetRotate = 0.0
        for target in cameraResult1.getTargets():
                id = target.getFiducialId()
                if id == 3:
                    targetFound = True
                    targetRotate = target.getYaw()

        launcherspeed = self.controls.launcher()
        self.launcher.set(launcherspeed * .62)

        x = self.controls.forward()
        y = self.controls.horizontal()
        rotate = self.controls.rotate()

        if self.controls.rotate_to_target() and targetFound:
            rotate = targetRotate

        fieldRelative = True
        if fieldRelative:
            gyroYaw = self.gyro.getAngle(wpilib.ADIS16470_IMU.IMUAxis.kYaw)
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

        if self.timer.hasElapsed(0.5):
            #print("left forward encoder pos" + str(self.left_forward.encoder.getAbsolutePosition()))
            #print("left rear encoder pos" + str(self.left_rear.encoder.getAbsolutePosition()))
            #print("right forward encoder pos" + str(self.right_forward.encoder.getAbsolutePosition()))
            #print("right rear encoder pos" + str(self.right_rear.encoder.getAbsolutePosition()))
            #print("Results: " + str(self.limelight1.getLatestResult()))

            self.timer.reset()



if __name__ == "__main__":
    wpilib.run(MyRobot)