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
import math
import movement

class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        self.kDefaultLauncherScale = 0.62
        self.kDefaultScoopScale = 1.0
        self.kDefaultMiddleRampScale = 1.0

        self.nt = ntcore.NetworkTableInstance.getDefault()
        self.nt.startServer()
        self.smartdashboard = self.nt.getTable("SmartDashboard")
        self.smartdashboard.putNumber("launcher_speed", self.kDefaultLauncherScale)
        self.smartdashboard.putNumber("scoop_speed", self.kDefaultScoopScale)
        self.smartdashboard.putNumber("middle_ramp_speed", self.kDefaultMiddleRampScale)
        
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

        self.launcher = rev.CANSparkMax(6, rev._rev.CANSparkLowLevel.MotorType.kBrushless)
        self.launcher2 = rev.CANSparkMax(8, rev._rev.CANSparkLowLevel.MotorType.kBrushless)
        self.launcher2.follow(self.launcher, True)

        self.intakeScoop = rev.CANSparkMax(9, rev._rev.CANSparkLowLevel.MotorType.kBrushless)
        self.middleRamp = rev.CANSparkMax(7, rev._rev.CANSparkLowLevel.MotorType.kBrushless)

        self.limelight1 = photonCamera.PhotonCamera("limelight1")
        self.k_maxmisses = 5
        self.target = {3:{"target":None, "misses":self.k_maxmisses}}

        self.controls = controls.Controls(0, 1)
        self.timer = wpilib.Timer()

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        self.timer.reset()
        self.timer.start()

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        #m = self.GetCameraMovement()
        #self.Drive(m.forward, m.horizontal, m.rotate, False)

    def teleopInit(self):
        """This function is run once each time the robot enters teleop mode."""
        self.timer.reset()
        self.timer.start()

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        if self.controls.reset_gyro():
            self.gyro.reset()

        launcherscale = self.smartdashboard.getNumber("launcher_speed", self.kDefaultLauncherScale)
        launcherspeed = self.controls.launcher()
        self.launcher.set(launcherspeed * launcherscale)

        scoopscale = self.smartdashboard.getNumber("scoop_speed", self.kDefaultScoopScale)
        scoopspeed = self.controls.scoop_speed()
        self.intakeScoop.set(scoopspeed * scoopscale)

        middlescale = self.smartdashboard.getNumber("middle_ramp_speed", self.kDefaultMiddleRampScale)
        middlespeed = self.controls.middle_speed()
        self.middleRamp.set(middlespeed * middlescale)

        forward = self.controls.forward()
        horizontal = self.controls.horizontal()
        rotate = self.controls.rotate()
        fieldRelative = True

        # Overwrite movement from camera if we say so
        if self.controls.rotate_to_target():
            m = self.GetCameraMovement()
            rotate = m.rotate
            forward = m.forward
            horizontal = m.horizontal
            fieldRelative = False

        self.Drive(forward, horizontal, rotate, fieldRelative)

        if self.timer.hasElapsed(0.5):
            #print("Results: " + str(self.limelight1.getLatestResult()))
            self.timer.reset()

    def Drive(self, forward, horizontal, rotate, fieldRelative):
        if fieldRelative:
            gyroYaw = self.gyro.getAngle(wpilib.ADIS16470_IMU.IMUAxis.kYaw)
            relativeRotation = wpimath.geometry.Rotation2d.fromDegrees(gyroYaw)
            chassisSpeeds = wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(forward, horizontal , rotate, relativeRotation)
        else:
            chassisSpeeds = wpimath.kinematics.ChassisSpeeds(forward, horizontal, rotate)

        self.driveTrain.Drive(chassisSpeeds, self.getPeriod())

    def GetCameraMovement(self):
        cameraResult1 = self.limelight1.getLatestResult()
        id = -1
        targetRotate = 0.0
        targetRotateRadians = 0
        targetPose = wpimath.geometry.Transform2d()
        for target in cameraResult1.getTargets():
                id = target.getFiducialId()
                self.target[id]["target"] = target 
                self.target[id]["misses"] = 0

        for i, value in self.target.items():
            if self.target[i]["misses"] < self.k_maxmisses:
                self.target[i]["misses"] += 1 
                id = i 
            else: 
                self.target[i]["target"] = None

        if id == 3:
            targetRotate = self.target[id]["target"].getYaw() * -1
            targetRotateRadians = wpimath.units.degreesToRadians(targetRotate)
            targetPose = self.target[id]["target"].getBestCameraToTarget()
            

        rotate = targetRotateRadians * 0.2
        forward = targetPose.X() * -1
        horizontal = targetPose.Y() * -1

        # Stop about 2 meters away
        if abs(forward) < 1.25:
            forward = 0

        if abs(horizontal) < 0.5:
            horizontal = 0

        if abs(rotate) < 0.01:
            rotate = 0

        # Cap the speed
        forward = capValue(forward, 1)
        horizontal = capValue(horizontal, 1)
        rotate = capValue(rotate, math.pi)

        output=movement.Movement(forward, horizontal, rotate)
        return output
    
if __name__ == "__main__":
    wpilib.run(MyRobot)

def capValue(value, cap):
    if value > cap:
        return cap
    elif value < -1 * cap:
        return -1 * cap
    else:
        return value