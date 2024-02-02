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
import phoenix6
import phoenix5

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
        self.kMaxSpeed = 10.0 # meters per second
        self.kMaxRotate = 0.5 # radians? per second

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

        p_value = 6e-5
        i_value = 1e-6
        d_value = 0

        angle_p_value = 0.1
        
        # Front Left
        fldriveMotorParams = motorParams.Motorparams(22, p_value, i_value, d_value)
        flangleMotorParams = motorParams.Motorparams(23, angle_p_value)
        flEncoderParams = encoderParams.EncoderParams(32, -0.089844)
        flParams = swerve_drive_params.SwerveDriveParams(fldriveMotorParams, flangleMotorParams, flEncoderParams)

        # Rear Left
        rldriveMotorParams = motorParams.Motorparams(24, p_value, i_value, d_value)
        rlangleMotorParams = motorParams.Motorparams(25, angle_p_value)
        rlEncoderParams = encoderParams.EncoderParams(33, -0.094971)
        rlParams = swerve_drive_params.SwerveDriveParams(rldriveMotorParams, rlangleMotorParams, rlEncoderParams)

        # Front Right
        frdriveMotorParams = motorParams.Motorparams(20, p_value, i_value, d_value)
        frangleMotorParams = motorParams.Motorparams(21, angle_p_value)
        frEncoderParams = encoderParams.EncoderParams(31, 0.037842)
        frParams = swerve_drive_params.SwerveDriveParams(frdriveMotorParams, frangleMotorParams, frEncoderParams)
        
        # Rear Right
        rrdriveMotorParams = motorParams.Motorparams(26, p_value, i_value, d_value)
        rrangleMotorParams = motorParams.Motorparams(27, angle_p_value)
        rrEncoderParams = encoderParams.EncoderParams(30, -0.236328)
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

        self.climber = phoenix5.TalonFX(10)

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
        self.cameraTimer = wpilib.Timer()
        self.cameraTimer.start()

    def robotPeriodic(self):
        swerveModulePositions = self.driveTrain.getSwerveModulePositions()
        rotation = wpimath.geometry.Rotation2d.fromDegrees(self.GetRotation())

        self.lastCameraPose = self.poseEstimator.update()
        ambiguity = 0
        if self.lastCameraPose is not None:
            cameraPose = self.lastCameraPose.estimatedPose.toPose2d()
            if len(self.lastCameraPose.targetsUsed) > 0:
                ambiguity = self.lastCameraPose.targetsUsed[0].getPoseAmbiguity()
                if ambiguity == 0 and self.cameraTimer.hasElapsed(5):
                    self.driveTrain.odometry.resetPosition(rotation, swerveModulePositions, cameraPose)
                    self.cameraTimer.reset()


        self.lastOdometryPose = self.driveTrain.odometry.update(rotation, swerveModulePositions)
        
        self.smartdashboard.putNumber("odometryX", self.lastOdometryPose.X())
        self.smartdashboard.putNumber("odometryY", self.lastOdometryPose.Y())
        self.smartdashboard.putNumber("rotation", self.lastOdometryPose.rotation().radians())
        self.smartdashboard.putNumber("ambiguity", ambiguity)

        self.smartdashboard.putNumber("FL_RPM", self.driveTrain.fl.speedRPM)
        self.smartdashboard.putNumber("FR_RPM", self.driveTrain.fr.speedRPM)
        self.smartdashboard.putNumber("RL_RPM", self.driveTrain.rl.speedRPM)
        self.smartdashboard.putNumber("RR_RPM", self.driveTrain.rr.speedRPM)

        self.smartdashboard.putNumber("FL_Speed", self.driveTrain.fl.speed)
        self.smartdashboard.putNumber("FR_Speed", self.driveTrain.fr.speed)
        self.smartdashboard.putNumber("RL_Speed", self.driveTrain.rl.speed)
        self.smartdashboard.putNumber("RR_Speed", self.driveTrain.rr.speed)

        self.smartdashboard.putNumber("FL_Velocity", self.driveTrain.fl.get_drive_velocity())
        self.smartdashboard.putNumber("FR_Velocity", self.driveTrain.fr.get_drive_velocity())
        self.smartdashboard.putNumber("RL_Velocity", self.driveTrain.rl.get_drive_velocity())
        self.smartdashboard.putNumber("RR_Velocity", self.driveTrain.rr.get_drive_velocity())

        
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

        forward = self.controls.forward() * self.kMaxSpeed
        horizontal = self.controls.horizontal() * self.kMaxSpeed
        rotate = self.controls.rotate()
        pose = wpimath.geometry.Pose2d(forward, horizontal, rotate)
        fieldRelative = True

        if self.controls.reset_goal():
            self.smartdashboard.putNumber("goalX", self.driveTrain.odometry.getPose().X())
            self.smartdashboard.putNumber("goalY", self.driveTrain.odometry.getPose().Y())
            self.smartdashboard.putNumber("goalR", self.driveTrain.odometry.getPose().rotation().radians())

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

        self.climber.set(phoenix5.TalonFXControlMode.PercentOutput, self.controls.climber())

        if self.timer.hasElapsed(0.5):
            #print("Results: " + str(self.limelight1.getLatestResult()))
            #print("Pose - " + str(self.lastPose))
            self.timer.reset()

    def Drive(self, pose: wpimath.geometry.Pose2d, fieldRelative):
        rotation = pose.rotation().radians()
        rotation *= self.kMaxRotate
        if fieldRelative:
            gyroYaw = self.GetRotation()
            relativeRotation = wpimath.geometry.Rotation2d.fromDegrees(gyroYaw)
            chassisSpeeds = wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(pose.X(), pose.Y() , rotation, relativeRotation)
        else:
            chassisSpeeds = wpimath.kinematics.ChassisSpeeds(pose.X(), pose.Y(), rotation)

        self.driveTrain.Drive(chassisSpeeds, self.getPeriod())

    def MoveToPose2d(self, pose: wpimath.geometry.Pose2d):
        trajectory = pose.relativeTo(self.lastOdometryPose)
        rotation = pose.rotation().radians() - self.lastOdometryPose.rotation().radians()

        self.smartdashboard.putNumber("trajectoryX", trajectory.X())
        self.smartdashboard.putNumber("trajectoryY", trajectory.Y())
        self.smartdashboard.putNumber("trajectoryR", rotation)

        rotate = rotation * self.kMaxRotate
        forward = trajectory.X() * self.kMaxSpeed
        horizontal = trajectory.Y() * self.kMaxSpeed

        if abs(forward) < 0.01:
            forward = 0.0
        if abs(horizontal) < 0.01:
            horizontal = 0.0
        if abs(rotate) < 0.001:
            rotate = 0.0

        # Cap the speed
        forward = capValue(forward, self.kMaxSpeed)
        horizontal = capValue(horizontal, self.kMaxSpeed)
        rotate = capValue(rotate, self.kMaxRotate)

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