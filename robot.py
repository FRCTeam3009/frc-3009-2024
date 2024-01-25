#!/usr/bin/env python3

import wpilib
import wpilib.drive
import wpimath.kinematics
import wpimath.geometry
import wpimath.units
import wpimath.filter
import controls
import swerve_drive_params
import drive_train
import rev
import sys
import test_imu
import motorParams
import encoderParams
from photonlibpy import photonCamera
import ntcore
import movement

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

        self.driveTrain = drive_train.DriveTrain(flParams, frParams, rlParams, rrParams)

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
        m = self.GetCameraMovement()
        self.Drive(m.forward, m.horizontal, m.rotate)

    def teleopInit(self):
        """This function is run once each time the robot enters teleop mode."""
        self.timer.reset()
        self.timer.start()

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        launcherspeed = self.controls.launcher()
        self.launcher.set(launcherspeed * .62)

        forward = self.controls.forward()
        horizontal = self.controls.horizontal()
        rotate = self.controls.rotate()

        # Overwrite movement from camera if we say so
        if self.controls.rotate_to_target():
            m = self.GetCameraMovement()
            rotate = m.rotate
            forward = m.forward
            horizontal = m.horizontal

        self.Drive(forward, horizontal, rotate)

        if self.timer.hasElapsed(0.5):
            #print("Results: " + str(self.limelight1.getLatestResult()))
            self.timer.reset()

    def Drive(self, forward, horizontal, rotate):
        fieldRelative = True
        if fieldRelative:
            gyroYaw = self.gyro.getAngle(wpilib.ADIS16470_IMU.IMUAxis.kYaw)
            relativeRotation = wpimath.geometry.Rotation2d.fromDegrees(gyroYaw)
            chassisSpeeds = wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(forward, horizontal , rotate, relativeRotation)
        else:
            chassisSpeeds = wpimath.kinematics.ChassisSpeeds(forward, horizontal, rotate)

        self.driveTrain.Drive(chassisSpeeds, self.getPeriod())

    def GetCameraMovement(self):
        cameraResult1 = self.limelight1.getLatestResult()

        targetRotate = 0.0
        targetRotateRadians = 0
        targetPose = wpimath.geometry.Transform2d()
        for target in cameraResult1.getTargets():
                id = target.getFiducialId()
                if id == 3:
                    targetRotate = target.getYaw() * -1
                    targetRotateRadians = wpimath.units.degreesToRadians(targetRotate)
                    targetPose = target.getBestCameraToTarget()

        targetRotateRadians *= 0.2
        rotate = targetRotateRadians
        forward = targetPose.X() * -1 * 0.1
        horizontal = targetPose.Y() * -1 * 0.1

        output=movement.Movement(forward, horizontal, rotate)
        return output
    
if __name__ == "__main__":
    wpilib.run(MyRobot)