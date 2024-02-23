#!/usr/bin/env python3

import wpilib
import commands2
import wpilib.drive
import wpimath.kinematics
import wpimath.geometry
import wpimath.units
import wpimath.filter
import wpilib.simulation
import controls
import swerve_drive_params
import drive_train
import chassis
import rev
import sys
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
import constants

# TODO ===FIRST===
# TODO pathplanner

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

        self.ledStrips = led.LedStrips()

        self.kSubwoofertags = [3, 4, 7, 8]
        self.kAmptags = [5, 6] 
        self.kStagetags = [11, 12, 13, 14, 15, 16]
        self.kSourcetags = [1, 2, 9, 10]
        self.kAlltags=[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]
        self.kDefaultScoopScale = 0.5
        self.kDefaultMiddleRampScale = 1.0
        self.kSubwooferDistance = 36.17 # inches
        self.kSubwooferStopDistance = self.kSubwooferDistance + 11.0

        self.lastOdometryPose = wpimath.geometry.Pose2d()

        self.nt = ntcore.NetworkTableInstance.getDefault()
        self.nt.startServer()
        self.smartdashboard = self.nt.getTable("SmartDashboard")
        self.smartdashboard.putNumber("scoop_speed", self.kDefaultScoopScale)
        self.smartdashboard.putNumber("middle_ramp_speed", self.kDefaultMiddleRampScale)
        self.smartdashboard.putNumber("amp_distance", 0.5)

        self.smartdashboard.putNumber("servo open", constants.ServoOpen)
        self.smartdashboard.putNumber("servo closed", constants.ServoClosed)

        self.NoteCam = self.nt.getTable("NoteCam")
        self.ATagCam = self.nt.getTable("ATagCam")

        self.trapServo = wpilib.Servo(constants.Servo)
        self.trapServo.set(constants.ServoClosed)

        p_value = 6e-5
        i_value = 1e-6
        d_value = 0

        angle_p_value = 0.5
        self.chassisSpeeds = wpimath.kinematics.ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, 0, wpimath.geometry.Rotation2d())
        
        # Front Left
        fldriveMotorParams = motorParams.Motorparams(GetCanId(constants.FLDrive), p_value, i_value, d_value)
        flangleMotorParams = motorParams.Motorparams(GetCanId(constants.FLAngle), angle_p_value)
        flEncoderParams = encoderParams.EncoderParams(GetCanId(constants.FLEncoder), constants.FLEncoderOffset)
        flParams = swerve_drive_params.SwerveDriveParams(fldriveMotorParams, flangleMotorParams, flEncoderParams)

        # Rear Left
        rldriveMotorParams = motorParams.Motorparams(GetCanId(constants.RLDrive), p_value, i_value, d_value)
        rlangleMotorParams = motorParams.Motorparams(GetCanId(constants.RLAngle), angle_p_value)
        rlEncoderParams = encoderParams.EncoderParams(GetCanId(constants.RLEncoder), constants.RLEncoderOffset)
        rlParams = swerve_drive_params.SwerveDriveParams(rldriveMotorParams, rlangleMotorParams, rlEncoderParams)

        # Front Right
        frdriveMotorParams = motorParams.Motorparams(GetCanId(constants.FRDrive), p_value, i_value, d_value)
        frangleMotorParams = motorParams.Motorparams(GetCanId(constants.FRAngle), angle_p_value)
        frEncoderParams = encoderParams.EncoderParams(GetCanId(constants.FREncoder), constants.FREncoderOffset)
        frParams = swerve_drive_params.SwerveDriveParams(frdriveMotorParams, frangleMotorParams, frEncoderParams)
        
        # Rear Right
        rrdriveMotorParams = motorParams.Motorparams(GetCanId(constants.RRDrive), p_value, i_value, d_value)
        rrangleMotorParams = motorParams.Motorparams(GetCanId(constants.RRAngle), angle_p_value)
        rrEncoderParams = encoderParams.EncoderParams(GetCanId(constants.RREncoder), constants.RREncoderOffset)
        rrParams = swerve_drive_params.SwerveDriveParams(rrdriveMotorParams, rrangleMotorParams, rrEncoderParams)
            
        self.driveTrain = drive_train.DriveTrain(self.chassis, flParams, frParams, rlParams, rrParams, self.getPeriod())

        self.noteSensorBottom = wpilib.DigitalInput(constants.NoteSensorBottom)
        self.noteSensorTop = wpilib.DigitalInput(constants.NoteSensorTop)
        self.noteSensorFront = wpilib.DigitalInput(constants.NoteSensorFront)

        self.intakeScoop = rev.CANSparkMax(GetCanId(constants.Scoop), rev._rev.CANSparkLowLevel.MotorType.kBrushless)
        self.shooter = shooter.Shooter(
            GetCanId(constants.ShooterTop),
            GetCanId(constants.ShooterBottom),
            GetCanId(constants.Middle),
            self.noteSensorBottom,
            self.noteSensorTop,
            self.intakeScoop)

        self.climber = phoenix5.TalonFX(GetCanId(constants.Climber))
        cameraHeightConversion = wpimath.units.inchesToMeters(20.5)
        cameraXConversion = wpimath.units.inchesToMeters(13.5)
        cameraYConversion = wpimath.units.inchesToMeters(1)

        # -30 degrees
        robotToNoteCameraRotation = wpimath.geometry.Rotation3d(0, math.pi/-6, 0)
        self.robotToNoteCamera = wpimath.geometry.Transform3d(
            cameraXConversion,
            cameraYConversion,
            cameraHeightConversion,
            robotToNoteCameraRotation)
        
        cameraXConversion = wpimath.units.inchesToMeters(12.5)

        # 25 degrees
        robotToAprilCameraRotation = wpimath.geometry.Rotation3d(0, math.pi/7.2, 0)
        self.robotToAprilCamera = wpimath.geometry.Transform3d(
            cameraXConversion,
            cameraYConversion,
            cameraHeightConversion,
            robotToAprilCameraRotation)
        

        self.limelight1 = photonCamera.PhotonCamera("limelight1")
        self.k_maxmisses = 5
        self.target = {}
        for i in range(1,16):
            self.target[i] = {"target":None, "misses":self.k_maxmisses}

        self.aprilTagFieldLayout = robotpy_apriltag.loadAprilTagLayoutField(robotpy_apriltag.AprilTagField.k2024Crescendo)

        self.controls = controls.Controls(0, 1)
        self.timer = wpilib.Timer()
        self.startPoseTimer = wpilib.Timer()
        self.startPoseTimer.start()
        self.cameraTimer = wpilib.Timer()
        self.cameraTimer.start()

        pathplannerlib.auto.NamedCommands.registerCommand("shoot", TestCommand())

        self.txATag=0
        self.tyATag=0
        self.txNote=0
        self.tyNote=0
        self.botpose=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.startPose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.startPoselist = []
        self.startPoseCalibrating = True
        

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
        self.smartdashboard.putNumber("rotation", self.lastOdometryPose.rotation().degrees())

        self.smartdashboard.putNumber("FL_Velocity", self.driveTrain.fl.get_drive_velocity())
        self.smartdashboard.putNumber("FR_Velocity", self.driveTrain.fr.get_drive_velocity())
        self.smartdashboard.putNumber("RL_Velocity", self.driveTrain.rl.get_drive_velocity())
        self.smartdashboard.putNumber("RR_Velocity", self.driveTrain.rr.get_drive_velocity())

        self.smartdashboard.putNumber("chassis_speeds_vx", self.chassisSpeeds.vx)
        self.smartdashboard.putNumber("chassis_speeds_vy", self.chassisSpeeds.vy)
        self.smartdashboard.putNumber("chassis_speeds_omega", self.chassisSpeeds.omega)
        self.smartdashboard.putNumber("note sensor top", self.noteSensorTop.get())
        self.smartdashboard.putNumber("note sensor bottom", self.noteSensorBottom.get())
        self.smartdashboard.putNumber("note sensor front", self.noteSensorFront.get())

        constants.ServoOpen = self.smartdashboard.getNumber("servo open", constants.ServoOpen)
        constants.ServoClosed = self.smartdashboard.getNumber("servo closed", constants.ServoClosed)

        self.smartdashboard.putNumberArray("startpose", self.startPose)

        ATagCamTargetSeen = self.ATagCam.getNumber("tv",0)
        NoteCamTargetSeen = self.NoteCam.getNumber("tv",0)
        if NoteCamTargetSeen > 0:
            self.txATag=self.ATagCam.getNumber("tx",0)
            self.tyATag=self.ATagCam.getNumber("ty",0)

        if ATagCamTargetSeen > 0:
            self.txNote=self.NoteCam.getNumber("tx",0)
            self.tyNote=self.NoteCam.getNumber("ty",0)
            self.botpose=self.ATagCam.getEntry("botpose").getDoubleArray([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            if self.startPoseCalibrating:
                self.startPoselist.append(self.botpose)
                if self.startPoseTimer.hasElapsed(5):
                    self.startPoseCalibrating = False
                    self.startPose = averagePoses(self.startPoselist)
                    startPose2d = pose2dFromNTPose(self.startPose)
                    self.driveTrain.resetPosition(startPose2d)
                    self.smartdashboard.putNumberArray("startpose", self.startPose)

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        self.timer.reset()
        self.timer.start()

        self.driveTrain.UpdateMaxSpeed(0.1)

        self.trapServo.set(constants.ServoClosed)

        # TODO testing
        testRotate = wpimath.geometry.Rotation2d.fromDegrees(-62.02)
        testPose = wpimath.geometry.Pose2d(15.39, 2.01, testRotate)
        self.driveTrain.resetPosition(testPose)

        self.driveTrain.AutoInit()
        self.automode = PathPlannerAuto("ShortTest")
        self.automode.schedule()

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        # This function is empty because we're using the command scheduler to run our autonomous
        self.shooter.stop_motors()
        self.climber.set(phoenix5.TalonFXControlMode.PercentOutput, 0)

    def teleopInit(self):
        """This function is run once each time the robot enters teleop mode."""
        self.driveTrain.UpdateMaxSpeed(constants.MaxSpeed)
        self.trapServo.set(constants.ServoClosed)

        commands2.CommandScheduler.getInstance().cancelAll()
        self.timer.reset()
        self.timer.start()

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        if self.controls.reset_gyro():
            self.driveTrain.gyro.reset()
        
        if self.controls.shootspeaker():
            self.shooter.fire(shooter.Shooter.speakerscale, self.controls.override(), self.controls.reverseOverride()) 
        elif self.controls.shootamp():
            self.shooter.fire(shooter.Shooter.ampscale, self.controls.override(), self.controls.reverseOverride())
        else: 
            self.shooter.stop()
        
        if self.controls.push_trap():
            if self.trapServo.get() < 0.15:
                self.trapServo.set(constants.ServoClosed)
            else:
                self.trapServo.set(constants.ServoOpen)

        
        forward = self.controls.forward() * self.getInputSpeed(self.driveTrain.maxSpeed)
        horizontal = self.controls.horizontal() * self.getInputSpeed(self.driveTrain.maxSpeed)
        rotate = self.controls.rotate() * self.getInputSpeed(self.driveTrain.maxRotate)
        pose = wpimath.geometry.Pose2d(forward, horizontal, rotate)
        fieldRelative = True

        # Overwrite movement from camera if we say so
        if self.controls.note_pickup():
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
            self.timer.reset()

    def Drive(self, pose: wpimath.geometry.Pose2d, fieldRelative):
         # Cap the speeds
        forward = capValue(pose.X(), self.driveTrain.maxSpeed)
        horizontal = capValue(pose.Y(), self.driveTrain.maxSpeed)
        rotate = capValue(pose.rotation().radians(), self.driveTrain.maxRotate)

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

        rotate = rotation * self.driveTrain.maxRotate
        forward = trajectory.X() * self.driveTrain.maxSpeed
        horizontal = trajectory.Y() * self.driveTrain.maxSpeed

        if abs(forward) < 0.01:
            forward = 0.0
        if abs(horizontal) < 0.01:
            horizontal = 0.0
        if abs(rotate) < 0.01:
            rotate = 0.0

        output = wpimath.geometry.Pose2d(forward, horizontal, rotate)
        return output
    
    def GetRotation(self):
        return self.driveTrain.GetRotation()
    
    def line_up_to_target(self, tag_list):
        tid = self.ATagCam.getEntry("tid").getDoubleArray(None)
        tagFound = tid is not None and len(tid) > 0
        if tagFound and tid[0] in tag_list:
            r = -self.txATag
            verticalAngle = self.tyATag - self.robotToAprilCamera.rotation().y_degrees
            x = -math.tan(verticalAngle) * self.robotToAprilCamera.Z()
            # if we're targeting the speaker adjust for the base
            if tid[0] in self.kSubwoofertags:
                x = x - 1.2
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
        goalX = math.tan(self.tyNote + self.robotToNoteCamera.rotation().y_degrees)*self.robotToNoteCamera.Z()
        goalY = 0.0
        goalRotation = self.txNote
        rotation = wpimath.geometry.Rotation2d(goalRotation)
        goal = wpimath.geometry.Pose2d(goalX, goalY, rotation)
        return self.MoveToPose2d(goal)
    
    def _simulationPeriodic(self):
        testAngle = 180
        self.driveTrain.gyroSim.setGyroAngleY(testAngle)
        self.driveTrain.gyro.setGyroAngleY(testAngle)
        rotate = wpimath.geometry.Rotation2d.fromDegrees(testAngle)
        current = self.driveTrain.odometry.getPose()
        self.driveTrain.odometry.resetPosition(rotate, self.driveTrain.getSwerveModulePositions(), current)
        
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
    
def averagePoses(poselist):
    output= [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    for pose in poselist:
        i = 0
        while i < 6:
            output[i] = output[i] + pose[i]
            i += 1

    n = len(poselist)
    i = 0
    while i < 6:
        output[i] = output[i] / n
        i += 1

    return output

def pose2dFromNTPose(ntPose) -> wpimath.geometry.Pose2d:
    if len(ntPose) != 6:
        return wpimath.geometry.Pose2d()

    x = wpimath.units.meters(ntPose[0])
    y = wpimath.units.meters(ntPose[1])
    z = wpimath.units.meters(ntPose[2]) # not needed, we can't fly
    rotate = wpimath.geometry.Rotation3d(ntPose[3], ntPose[4], ntPose[5])

    return wpimath.geometry.Pose2d(x, y, rotate.toRotation2d())
