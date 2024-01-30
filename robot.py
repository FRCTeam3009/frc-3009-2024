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
import robotpy_apriltag
from photonlibpy import photonCamera, photonPoseEstimator
import ntcore
import math
import movement

class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        self.kSubwoofertags = [3, 4, 7, 8]
        self.kAmptags = [5, 6, 2]# to do remove 2 
        self.kStagetags = [11, 12, 13, 14, 15, 16]
        self.kSourcetags = [1, 2, 9, 10]
        self.kDefaultLauncherScale = 0.62
        self.kDefaultScoopScale = 0.5
        self.kDefaultMiddleRampScale = 1.0

        self.goalPosition = 0.0
        self.currentPosition = 0.0
        self.closeApproach = False

        self.lastTarget = wpimath.geometry.Pose3d()
        self.lastOdometryPose = wpimath.geometry.Pose2d()
        self.lastCameraPose = wpimath.geometry.Pose2d()
        self.lastDistance = 0.0

        self.nt = ntcore.NetworkTableInstance.getDefault()
        self.nt.startServer()
        self.smartdashboard = self.nt.getTable("SmartDashboard")
        self.smartdashboard.putNumber("launcher_speed", self.kDefaultLauncherScale)
        self.smartdashboard.putNumber("scoop_speed", self.kDefaultScoopScale)
        self.smartdashboard.putNumber("middle_ramp_speed", self.kDefaultMiddleRampScale)
        self.smartdashboard.putNumber("amp_distance", 0.5)

        self.smartdashboard.putNumber("goalX", 0.0)
        self.smartdashboard.putNumber("goalY", 0.0)
        self.smartdashboard.putNumber("goalR", 0.0)
        
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

        robotToCameraRotation = wpimath.geometry.Rotation3d(0, 0, 0)
        self.robotToCamera = wpimath.geometry.Transform3d(
            wpimath.units.meters(0.0),
            wpimath.units.meters(0.0),
            wpimath.units.meters(0.0),
            robotToCameraRotation)

        self.limelight1 = photonCamera.PhotonCamera("limelight1")
        self.k_maxmisses = 5
        self.target = {}
        for i in range(1,16):
            self.target[i] = {"target":None, "misses":self.k_maxmisses}

        self.aprilTagFieldLayout = robotpy_apriltag.loadAprilTagLayoutField(robotpy_apriltag.AprilTagField.k2024Crescendo)
        self.poseEstimator = photonPoseEstimator.PhotonPoseEstimator(
            self.aprilTagFieldLayout,
            photonPoseEstimator.PoseStrategy(1),
            self.limelight1,
            self.robotToCamera,
        )

        self.controls = controls.Controls(0, 1)
        self.timer = wpilib.Timer()
    def robotPeriodic(self):
        swerveModulePositions = self.driveTrain.getSwerveModulePositions()
        rotation = wpimath.geometry.Rotation2d.fromDegrees(self.GetRotation())

        self.lastCameraPose = self.poseEstimator.update()
        if self.lastCameraPose is not None:
            cameraPose = self.lastCameraPose.estimatedPose.toPose2d()
            self.driveTrain.odometry.resetPosition(rotation, swerveModulePositions, cameraPose)

        self.lastOdometryPose = self.driveTrain.odometry.update(rotation, swerveModulePositions)
        
        self.smartdashboard.putNumber("odometryX", self.lastOdometryPose.X())
        self.smartdashboard.putNumber("odometryY", self.lastOdometryPose.Y())
        self.smartdashboard.putNumber("rotation", self.lastOdometryPose.rotation().radians())

        
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
        pose = wpimath.geometry.Pose2d(forward, horizontal, rotate)
        fieldRelative = True

        # Overwrite movement from camera if we say so
        if self.controls.rotate_to_target():
            goalX = self.smartdashboard.getNumber("goalX", 0.0)
            goalY = self.smartdashboard.getNumber("goalY", 0.0)
            goalRotation = self.smartdashboard.getNumber("goalR", 0.0)
            rotation = wpimath.geometry.Rotation2d(goalRotation)
            goal = wpimath.geometry.Pose2d(goalX, goalY, rotation)
            pose = self.MoveToPose2d(goal)
            fieldRelative = False

        self.Drive(pose, fieldRelative)

        if self.timer.hasElapsed(0.5):
            #print("Results: " + str(self.limelight1.getLatestResult()))
            #print("Pose - " + str(self.lastPose))
            self.timer.reset()

    def Drive(self, pose: wpimath.geometry.Pose2d, fieldRelative):
        if fieldRelative:
            gyroYaw = self.GetRotation()
            relativeRotation = wpimath.geometry.Rotation2d.fromDegrees(gyroYaw)
            chassisSpeeds = wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(pose.X(), pose.Y() , pose.rotation().radians(), relativeRotation)
        else:
            chassisSpeeds = wpimath.kinematics.ChassisSpeeds(pose.X(), pose.Y(), pose.rotation().radians())

        self.driveTrain.Drive(chassisSpeeds, self.getPeriod())

    def MoveToPose2d(self, pose: wpimath.geometry.Pose2d):
        #trajectory = self.lastOdometryPose.relativeTo(pose)
        trajectory = pose.relativeTo(self.lastOdometryPose)

        self.smartdashboard.putNumber("trajectoryX", trajectory.X())
        self.smartdashboard.putNumber("trajectoryY", trajectory.Y())
        self.smartdashboard.putNumber("trajectoryR", trajectory.rotation().radians())

        rotate = trajectory.rotation().radians() * 0.3 # TODO pid controller
        forward = trajectory.X()
        horizontal = trajectory.Y()

        if abs(forward) < 0.01:
            forward = 0.0
        if abs(horizontal) < 0.01:
            horizontal = 0.0
        if abs(rotate) < 0.01:
            rotate = 0.0

        # Cap the speed
        forward = capValue(forward, 1)
        horizontal = capValue(horizontal, 1)
        rotate = capValue(rotate, math.pi)

        output = wpimath.geometry.Pose2d(forward, horizontal, rotate)
        return output
    
    def GetRotation(self):
        return self.gyro.getAngle(wpilib.ADIS16470_IMU.IMUAxis.kYaw)
    
if __name__ == "__main__":
    wpilib.run(MyRobot)

def capValue(value, cap):
    if value > cap:
        return cap
    elif value < -1 * cap:
        return -1 * cap
    else:
        return value