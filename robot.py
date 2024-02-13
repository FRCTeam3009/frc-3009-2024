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
import chassis
import rev
import sys
import test_imu
import motorParams
import encoderParams
import robotpy_apriltag
from photonlibpy import photonCamera, photonPoseEstimator
import ntcore
import phoenix5
import math
import shooter
from pathplannerlib.auto import PathPlannerAuto
import pathplannerlib.auto

# TODO ===FIRST===
# TODO pathplanner
# TODO average startup position using camera position

# TODO ===Dependencies Required===
# TODO add trap opener push thingy (requires the assembled thingy)
# TODO (extra) coral machine learning vision for notes (needs ML usb thing)

# TODO ===Last===
# TODO teach the team how the robot works so they can explain it to judges

TestCanId = 0

class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        self.chassis = chassis.Chassis()
        self.leds = wpilib.AddressableLED(5)
        self.ledLength = 151
        self.leds.setLength(self.ledLength)
        self.ledLit = 0
        self.ledBuff = []
        for led in range(self.ledLength):
            self.ledBuff.append(self.leds.LEDData())
            led_data = self.ledBuff[led]
            led_data.setRGB(0,0,0)
        self.leds.setData(self.ledBuff)
        self.leds.start()

        self.kSubwoofertags = [3, 4, 7, 8]
        self.kAmptags = [5, 6] 
        self.kStagetags = [11, 12, 13, 14, 15, 16]
        self.kSourcetags = [1, 2, 9, 10]
        self.kAlltags=[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]
        self.kDefaultScoopScale = 0.5
        self.kDefaultMiddleRampScale = 1.0
        self.kSubwooferDistance = 36.17 # inches
        self.kSubwooferStopDistance = self.kSubwooferDistance + 11.0

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
        self.smartdashboard.putNumber("scoop_speed", self.kDefaultScoopScale)
        self.smartdashboard.putNumber("middle_ramp_speed", self.kDefaultMiddleRampScale)
        self.smartdashboard.putNumber("amp_distance", 0.5)

        self.smartdashboard.putNumber("goalX", 0.0)
        self.smartdashboard.putNumber("goalY", 0.0)
        self.smartdashboard.putNumber("goalR", 0.0)

        p_value = 6e-5
        i_value = 1e-6
        d_value = 0

        angle_p_value = 6e-5
        self.something = 0.0
        self.chassisSpeeds = wpimath.kinematics.ChassisSpeeds.fromRobotRelativeSpeeds(0,0,0,wpimath.geometry.Rotation2d())
        
        # Front Left
        fldriveMotorParams = motorParams.Motorparams(GetCanId(22), p_value, i_value, d_value)
        flangleMotorParams = motorParams.Motorparams(GetCanId(23), angle_p_value)
        flEncoderParams = encoderParams.EncoderParams(GetCanId(32), -0.089844)
        flParams = swerve_drive_params.SwerveDriveParams(fldriveMotorParams, flangleMotorParams, flEncoderParams)

        # Rear Left
        rldriveMotorParams = motorParams.Motorparams(GetCanId(24), p_value, i_value, d_value)
        rlangleMotorParams = motorParams.Motorparams(GetCanId(25), angle_p_value)
        rlEncoderParams = encoderParams.EncoderParams(GetCanId(33), -0.094971)
        rlParams = swerve_drive_params.SwerveDriveParams(rldriveMotorParams, rlangleMotorParams, rlEncoderParams)

        # Front Right
        frdriveMotorParams = motorParams.Motorparams(GetCanId(20), p_value, i_value, d_value)
        frangleMotorParams = motorParams.Motorparams(GetCanId(21), angle_p_value)
        frEncoderParams = encoderParams.EncoderParams(GetCanId(31), 0.037842)
        frParams = swerve_drive_params.SwerveDriveParams(frdriveMotorParams, frangleMotorParams, frEncoderParams)
        
        # Rear Right
        rrdriveMotorParams = motorParams.Motorparams(GetCanId(26), p_value, i_value, d_value)
        rrangleMotorParams = motorParams.Motorparams(GetCanId(27), angle_p_value)
        rrEncoderParams = encoderParams.EncoderParams(GetCanId(30), -0.236328)
        rrParams = swerve_drive_params.SwerveDriveParams(rrdriveMotorParams, rrangleMotorParams, rrEncoderParams)

        self.gyro = test_imu.TestIMU()
        if "pytest" not in sys.modules:
            self.gyro = wpilib.ADIS16470_IMU()
            
        self.driveTrain = drive_train.DriveTrain(self.chassis, flParams, frParams, rlParams, rrParams, self.getPeriod(), self.gyro)

        self.noteSensorBottom = wpilib.DigitalInput(9)
        self.noteSensorTop = wpilib.DigitalInput(8)
        self.intakeScoop = rev.CANSparkMax(GetCanId(9), rev._rev.CANSparkLowLevel.MotorType.kBrushless)
        self.shooter = shooter.Shooter(GetCanId(6), GetCanId(8), GetCanId(7), self.noteSensorBottom, self.noteSensorTop, self.intakeScoop)

        self.climber = phoenix5.TalonFX(GetCanId(10))

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

        pathplannerlib.auto.NamedCommands.registerCommand("shoot", TestCommand())
        

    def robotPeriodic(self):
        for led in range(self.ledLength):
            led_data = self.ledBuff[led]
            if self.ledLit % self.ledLength == led:
                led_data.setRGB(250,0,0)
            else:
                led_data.setRGB(0,0,250)
        self.ledLit += 1
        self.leds.setData(self.ledBuff)

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

        self.smartdashboard.putNumber("FL_FF", self.driveTrain.fl.driveFF)
        self.smartdashboard.putNumber("FR_FF", self.driveTrain.fr.driveFF)
        self.smartdashboard.putNumber("RL_FF", self.driveTrain.rl.driveFF)
        self.smartdashboard.putNumber("RR_FF", self.driveTrain.rr.driveFF)

        self.smartdashboard.putNumber("FL_Velocity", self.driveTrain.fl.get_drive_velocity())
        self.smartdashboard.putNumber("FR_Velocity", self.driveTrain.fr.get_drive_velocity())
        self.smartdashboard.putNumber("RL_Velocity", self.driveTrain.rl.get_drive_velocity())
        self.smartdashboard.putNumber("RR_Velocity", self.driveTrain.rr.get_drive_velocity())

        self.smartdashboard.putNumber("motor_factor", self.driveTrain.fl._chassis._driveMotorConversionFactor)
        self.smartdashboard.putNumber("rotate_factor", self.driveTrain.fl._chassis._angleMotorConversionFactor)
        self.smartdashboard.putNumber("max_speed", self.driveTrain.kMaxSpeed)
        self.smartdashboard.putNumber("max_rotate", self.driveTrain.kMaxRotate)
        self.smartdashboard.putNumber("something_rotation", self.something)
        self.smartdashboard.putNumber("chassis_speeds_vx", self.chassisSpeeds.vx)
        self.smartdashboard.putNumber("chassis_speeds_vy", self.chassisSpeeds.vy)
        self.smartdashboard.putNumber("chassis_speeds_omega", self.chassisSpeeds.omega)
        self.smartdashboard.putNumber("note sensor", self.noteSensorTop.get())


    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        self.timer.reset()
        self.timer.start()

        self.driveTrain.AutoInit()
        self.automode = PathPlannerAuto("rightSpeakerBLUE")
        self.automode.initialize()

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        #m = self.GetCameraMovement()
        #self.Drive(m.forward, m.horizontal, m.rotate, False)
        self.automode.execute()

    def teleopInit(self):
        """This function is run once each time the robot enters teleop mode."""
        self.timer.reset()
        self.timer.start()

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        if self.controls.reset_gyro():
            self.gyro.reset()
        
        if self.controls.shootspeaker():
            self.shooter.fire(shooter.Shooter.speakerscale) 
        elif self.controls.shootamp():
            self.shooter.fire(shooter.Shooter.ampscale)
        else: 
            self.shooter.stop()

        
        forward = self.controls.forward() * self.getInputSpeed(self.driveTrain.kMaxSpeed)
        horizontal = self.controls.horizontal() * self.getInputSpeed(self.driveTrain.kMaxSpeed)
        rotate = self.controls.rotate() * self.getInputSpeed(self.driveTrain.kMaxRotate)
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
        elif self.controls.target_amp():
            pose = self.line_up_to_target(self.kAmptags)
            fieldRelative = False
        elif self.controls.target_subwoofer():
            pose = self.line_up_to_target(self.kSubwoofertags)
            magnitude = math.sqrt(pose.X()**2 + pose.Y()**2)
            if magnitude < self.kSubwooferStopDistance:
                pose = wpimath.geometry.Pose2d()
            fieldRelative = False
        elif self.controls.target_stage():
            pose = self.line_up_to_target(self.kStagetags)
            fieldRelative = False
        elif self.controls.target_closest():
            pose = self.line_up_to_target(self.kAlltags)
            fieldRelative = False

        self.Drive(pose, fieldRelative)

        self.climber.set(phoenix5.TalonFXControlMode.PercentOutput, self.controls.climber())

        if self.timer.hasElapsed(0.5):
            #print("Results: " + str(self.limelight1.getLatestResult()))
            #print("Pose - " + str(self.lastPose))
            self.timer.reset()

    def Drive(self, pose: wpimath.geometry.Pose2d, fieldRelative):
         # Cap the speeds
        forward = capValue(pose.X(), self.driveTrain.kMaxSpeed)
        horizontal = capValue(pose.Y(), self.driveTrain.kMaxSpeed)
        rotate = capValue(pose.rotation().radians(), self.driveTrain.kMaxRotate)
        self.something = pose.rotation().radians()

        gyroYaw = self.GetRotation()
        relativeRotation = wpimath.geometry.Rotation2d.fromDegrees(gyroYaw)

        chassisSpeeds = wpimath.kinematics.ChassisSpeeds.fromRobotRelativeSpeeds(forward, horizontal, rotate, relativeRotation)
        if fieldRelative:
            chassisSpeeds = wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(forward, horizontal, rotate, relativeRotation)

        self.chassisSpeeds = wpimath.kinematics.ChassisSpeeds.discretize(chassisSpeeds, self.getPeriod())
        self.driveTrain.Drive(chassisSpeeds)

    def MoveToPose2d(self, pose: wpimath.geometry.Pose2d):
        trajectory = pose.relativeTo(self.lastOdometryPose)
        rotation = pose.rotation().radians() - self.lastOdometryPose.rotation().radians()

        self.smartdashboard.putNumber("trajectoryX", trajectory.X())
        self.smartdashboard.putNumber("trajectoryY", trajectory.Y())
        self.smartdashboard.putNumber("trajectoryR", rotation)

        rotate = rotation * self.driveTrain.kMaxRotate
        forward = trajectory.X() * self.driveTrain.kMaxSpeed
        horizontal = trajectory.Y() * self.driveTrain.kMaxSpeed

        if abs(forward) < 0.01:
            forward = 0.0
        if abs(horizontal) < 0.01:
            horizontal = 0.0
        if abs(rotate) < 0.01:
            rotate = 0.0

        output = wpimath.geometry.Pose2d(forward, horizontal, rotate)
        return output
    
    def GetRotation(self):
        return self.gyro.getAngle(wpilib.ADIS16470_IMU.IMUAxis.kYaw)
    
    def line_up_to_target(self, tag_list):
        if self.lastCameraPose is not None:
            for targetSeen in self.lastCameraPose.targetsUsed:
                for targetid in tag_list:
                    if targetid == targetSeen.getFiducialId():
                        transform = targetSeen.getBestCameraToTarget()
                        rotate = targetSeen.getYaw()
                        rotation = wpimath.geometry.Rotation2d.fromDegrees(rotate)
                        pose = wpimath.geometry.Pose2d(transform.X(), transform.Y(), rotation)
                        return pose
                
        return wpimath.geometry.Pose2d()
    

    def getInputSpeed(self, speed):
        if self.controls.turbo():
            return speed
        elif self.controls.slow():
            return 0.25 * speed
        else:
            return 0.75 * speed
        
class TestCommand(pathplannerlib.auto.Command):
    def execute(self):
        print("SHOOTING")
        return super().execute()

if __name__ == "__main__":
    wpilib.run(MyRobot)

def capValue(value, cap):
    if value > cap:
        return cap
    elif value < -1 * cap:
        return -1 * cap
    else:
        return value
    
def GetCanId(id):
    global TestCanId
    if "pyfrc.tests" in sys.modules:
        TestCanId += 1
        return TestCanId
    else:
        return id