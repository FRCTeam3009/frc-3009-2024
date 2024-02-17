#!/usr/bin/env python3

import wpilib
import commands2
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
from photonlibpy import photonCamera
import ntcore
import phoenix5
import math
import shooter
from pathplannerlib.auto import PathPlannerAuto
import pathplannerlib.auto
import led
import ids

# TODO ===FIRST===
# TODO pathplanner
# TODO limelight global position
# TODO limelight visual servoing (auto-aim)

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

        self.ledStrips = led.ledStrips()

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

        self.NoteCam = self.nt.getTable("NoteCam")
        self.ATagCam = self.nt.getTable("ATagCam")

        self.trapServo = wpilib.Servo(ids.Servo)
        self.trapServo.set(0.5)

        p_value = 6e-5
        i_value = 1e-6
        d_value = 0

        angle_p_value = 6e-5
        self.something = 0.0
        self.chassisSpeeds = wpimath.kinematics.ChassisSpeeds.fromRobotRelativeSpeeds(0,0,0,wpimath.geometry.Rotation2d())
        
        # Front Left
        fldriveMotorParams = motorParams.Motorparams(GetCanId(ids.FLDrive), p_value, i_value, d_value)
        flangleMotorParams = motorParams.Motorparams(GetCanId(ids.FLAngle), angle_p_value)
        flEncoderParams = encoderParams.EncoderParams(GetCanId(ids.FLEncoder), -0.089844)
        flParams = swerve_drive_params.SwerveDriveParams(fldriveMotorParams, flangleMotorParams, flEncoderParams)

        # Rear Left
        rldriveMotorParams = motorParams.Motorparams(GetCanId(ids.RLDrive), p_value, i_value, d_value)
        rlangleMotorParams = motorParams.Motorparams(GetCanId(ids.RLAngle), angle_p_value)
        rlEncoderParams = encoderParams.EncoderParams(GetCanId(ids.RLEncoder), -0.094971)
        rlParams = swerve_drive_params.SwerveDriveParams(rldriveMotorParams, rlangleMotorParams, rlEncoderParams)

        # Front Right
        frdriveMotorParams = motorParams.Motorparams(GetCanId(ids.FRDrive), p_value, i_value, d_value)
        frangleMotorParams = motorParams.Motorparams(GetCanId(ids.FRAngle), angle_p_value)
        frEncoderParams = encoderParams.EncoderParams(GetCanId(ids.FREncoder), 0.037842)
        frParams = swerve_drive_params.SwerveDriveParams(frdriveMotorParams, frangleMotorParams, frEncoderParams)
        
        # Rear Right
        rrdriveMotorParams = motorParams.Motorparams(GetCanId(ids.RRDrive), p_value, i_value, d_value)
        rrangleMotorParams = motorParams.Motorparams(GetCanId(ids.RRAngle), angle_p_value)
        rrEncoderParams = encoderParams.EncoderParams(GetCanId(ids.RREncoder), -0.236328)
        rrParams = swerve_drive_params.SwerveDriveParams(rrdriveMotorParams, rrangleMotorParams, rrEncoderParams)

        self.gyro = test_imu.TestIMU()
        if "pytest" not in sys.modules:
            self.gyro = wpilib.ADIS16470_IMU()
            
        self.driveTrain = drive_train.DriveTrain(self.chassis, flParams, frParams, rlParams, rrParams, self.getPeriod(), self.gyro)

        self.noteSensorBottom = wpilib.DigitalInput(ids.NoteSensorBottom)
        self.noteSensorTop = wpilib.DigitalInput(ids.NoteSensorTop)
        self.noteSensorFront = wpilib.DigitalInput(ids.NoteSensorFront)

        self.intakeScoop = rev.CANSparkMax(GetCanId(ids.Scoop), rev._rev.CANSparkLowLevel.MotorType.kBrushless)
        self.shooter = shooter.Shooter(
            GetCanId(ids.ShooterTop),
            GetCanId(ids.ShooterBottom),
            GetCanId(ids.Middle),
            self.noteSensorBottom,
            self.noteSensorTop,
            self.intakeScoop)

        self.climber = phoenix5.TalonFX(GetCanId(ids.Climber))

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

        self.controls = controls.Controls(0, 1)
        self.timer = wpilib.Timer()
        self.cameraTimer = wpilib.Timer()
        self.cameraTimer.start()

        pathplannerlib.auto.NamedCommands.registerCommand("shoot", TestCommand())

        self.txATag=0
        self.tyATag=0
        self.txNote=0
        self.tyNote=0
        self.botpose=None
        

    def robotPeriodic(self):

        commands2.CommandScheduler.getInstance().run()

        #self.ledStrips.solid(0, 250, 0)
        self.ledStrips.gbRotate()

        swerveModulePositions = self.driveTrain.getSwerveModulePositions()
        rotation = wpimath.geometry.Rotation2d.fromDegrees(self.GetRotation())

        self.automode = None

        self.lastOdometryPose = self.driveTrain.odometry.update(rotation, swerveModulePositions)
        
        self.smartdashboard.putNumber("odometryX", self.lastOdometryPose.X())
        self.smartdashboard.putNumber("odometryY", self.lastOdometryPose.Y())
        self.smartdashboard.putNumber("rotation", self.lastOdometryPose.rotation().radians())

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

        ATagCamTargetSeen = self.ATagCam.getNumber("tv",0)
        NoteCamTargetSeen = self.NoteCam.getNumber("tv",0)
        if ATagCamTargetSeen > 0:
            self.txATag=self.ATagCam.getNumber("tx",0)
            self.tyATag=self.ATagCam.getNumber("ty",0)

        if NoteCamTargetSeen > 0:
            self.txNote=self.NoteCam.getNumber("tx",0)
            self.tyNote=self.NoteCam.getNumber("ty",0)
            self.botpose=self.ATagCam.getEntry("botpose").getDoubleArray(None)
    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        self.timer.reset()
        self.timer.start()

        self.trapServo.set(0.5)

        self.driveTrain.AutoInit()
        self.automode = PathPlannerAuto("rightSpeakerBLUE")
        self.automode.schedule()

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        #this function exists for some reason

    def teleopInit(self):
        """This function is run once each time the robot enters teleop mode."""
        self.trapServo.set(0.5)

        commands2.CommandScheduler.getInstance().cancelAll()
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
        
        if self.controls.push_trap():
            if self.trapServo.get() > 0.75:
                self.trapServo.set(0.5)
            else:
                self.trapServo.set(1.0)

        
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
            pose = self.noteLineup()
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
        tid = self.ATagCam.getEntry("tid").getDoubleArray(None)
        if tid is not None and len(tid) > 0:
            if tid[0] in tag_list:
                r = self.txATag
                x = math.tan(self.tyATag) * self.robotToCamera.Z()
                return wpimath.geometry.Pose2d(x,0,r)
            
        return wpimath.geometry.Pose2d()
    

    def getInputSpeed(self, speed):
        if self.controls.turbo():
            return speed
        elif self.controls.slow():
            return 0.25 * speed
        else:
            return 0.75 * speed
        

    def testInit(self) -> None:
        commands2.CommandScheduler.getInstance().cancelAll()

    def noteLineup(self):
        goalX = math.tan(self.tyNote)*self.robotToCamera.Z()
        goalY = self.smartdashboard.getNumber("goalY", 0.0)
        goalRotation = self.txNote
        rotation = wpimath.geometry.Rotation2d(goalRotation)
        goal = wpimath.geometry.Pose2d(goalX, goalY, rotation)
        return self.MoveToPose2d(goal)
        
class TestCommand(pathplannerlib.auto.Command):
    def execute(self):
        print("SHOOTING")
    def isFinished(self):
        return True

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