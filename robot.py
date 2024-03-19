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
import ntcore
import phoenix5
import math
import shooter
from pathplannerlib.auto import PathPlannerAuto
import pathplannerlib.auto
import led
import constants
import pathPlanner
import json

TestCanId = 0

class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        self.chassis = chassis.Chassis()

        self.ledStrips = led.LedStrips()

        self.kSpeakerTags = [3, 4, 7, 8]
        self.kSpeakerCenterTags = [4, 7]
        self.kAmptags = [5, 6] 
        self.kStagetags = [11, 12, 13, 14, 15, 16]
        self.kSourcetags = [1, 2, 9, 10]
        self.kAlltags=[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]
        self.kDefaultScoopScale = 0.5
        self.kDefaultMiddleRampScale = 1.0
        self.kSubwooferDistance = wpimath.units.inchesToMeters(36.17)
        self.kSubwooferStopDistance = wpimath.units.inchesToMeters(self.kSubwooferDistance + 11.0)

        self.lastOdometryPose = wpimath.geometry.Pose2d()

        self.nt = ntcore.NetworkTableInstance.getDefault()
        self.nt.startServer()
        self.smartdashboard = self.nt.getTable("SmartDashboard")
        self.smartdashboard.putNumber("scoop_speed", self.kDefaultScoopScale)
        self.smartdashboard.putNumber("middle_ramp_speed", self.kDefaultMiddleRampScale)
        self.smartdashboard.putNumber("amp_distance", 0.5)

        self.smartdashboard.putNumber("servo open", constants.ServoOpen)
        self.smartdashboard.putNumber("servo closed", constants.ServoClosed)

        pathPlanner.autonomousDropdown(self.smartdashboard)

        self.NoteCam = self.nt.getTable("limelight-front")
        self.ATagCam = self.nt.getTable("limelight-back")

        self.trapServo = wpilib.Servo(constants.Servo)
        self.trapServo.set(constants.ServoClosed)

        self.pitchServo = wpilib.Servo(constants.pitchServo)
        self.pitchServo.set(constants.speakerAngle)

        self.potInput = wpilib.AnalogInput(0)
        #self.potInput.setAverageBits(2)
        self.shooterPot = wpilib.AnalogPotentiometer(self.potInput, 180, -25) # device, servo range, pot at 0v location

        p_value = 6e-5
        i_value = 1e-6
        d_value = 0

        angle_p_value = 0.5
        
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
        self.autoToggle = wpilib.DigitalInput(constants.autoSwitch)

        self.intakeScoop = rev.CANSparkMax(GetCanId(constants.Scoop), rev._rev.CANSparkLowLevel.MotorType.kBrushless)
        self.intakeScoop.setInverted(True)
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

        pathplannerlib.auto.NamedCommands.registerCommand("shootSpeaker", pathPlanner.shootCommand(self.shooter, shooter.Shooter.speakerscale))
        pathplannerlib.auto.NamedCommands.registerCommand("shootAmp", pathPlanner.shootCommand(self.shooter, shooter.Shooter.ampscale))
        pathplannerlib.auto.NamedCommands.registerCommand("targetSpeaker", pathPlanner.lineAprilCommand(self.driveTrain, self.line_up_to_target, self.kSpeakerTags))
        pathplannerlib.auto.NamedCommands.registerCommand("targetAmp", pathPlanner.lineAprilCommand(self.driveTrain, self.line_up_to_target, self.kAmptags))
        pathplannerlib.auto.NamedCommands.registerCommand("targetClosest", pathPlanner.lineAprilCommand(self.driveTrain, self.line_up_to_target, self.kAlltags))
        pathplannerlib.auto.NamedCommands.registerCommand("targetNote", pathPlanner.lineNoteCommand(self.driveTrain, self.noteLineup, self.shooter))

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

        self.automode = None

        self.lastOdometryPose = self.driveTrain.Update()

        self.smartdashboard.putNumber("pot", self.shooterPot.get())   
        self.smartdashboard.putNumber("odometryX", self.lastOdometryPose.X())
        self.smartdashboard.putNumber("odometryY", self.lastOdometryPose.Y())
        self.smartdashboard.putNumber("odometryR", self.lastOdometryPose.rotation().degrees())

        self.smartdashboard.putNumber("FL_Velocity", self.driveTrain.fl.get_drive_velocity())
        self.smartdashboard.putNumber("FR_Velocity", self.driveTrain.fr.get_drive_velocity())
        self.smartdashboard.putNumber("RL_Velocity", self.driveTrain.rl.get_drive_velocity())
        self.smartdashboard.putNumber("RR_Velocity", self.driveTrain.rr.get_drive_velocity())

        self.smartdashboard.putNumber("chassis_speeds_vx", self.driveTrain.chassisSpeeds.vx)
        self.smartdashboard.putNumber("chassis_speeds_vy", self.driveTrain.chassisSpeeds.vy)
        self.smartdashboard.putNumber("chassis_speeds_omega", self.driveTrain.chassisSpeeds.omega)
        self.smartdashboard.putNumber("note sensor top", self.noteSensorTop.get())
        self.smartdashboard.putNumber("note sensor bottom", self.noteSensorBottom.get())
        self.smartdashboard.putNumber("note sensor front", self.noteSensorFront.get())
        
        self.smartdashboard.putNumber("speakerspeed",shooter.Shooter.speakerscale)
        self.smartdashboard.putNumber("ampspeed",shooter.Shooter.ampscale)
        self.smartdashboard.putNumber("trapspeed",shooter.Shooter.trapscale)

        self.smartdashboard.putBoolean("hasNote", self.shooter.hasNote())
        self.smartdashboard.putNumber("autoMode", 0)

        self.driveTrain.publishDashboardStates(self.smartdashboard)

        constants.ServoOpen = self.smartdashboard.getNumber("servo open", constants.ServoOpen)
        constants.ServoClosed = self.smartdashboard.getNumber("servo closed", constants.ServoClosed)

        self.smartdashboard.putNumberArray("startpose", self.startPose)

        ATagCamTargetSeen = self.ATagCam.getNumber("tv",0)
        NoteCamTargetSeen = self.NoteCam.getNumber("tv",0)
        self.txNote=self.NoteCam.getNumber("tx",0)
        self.tyNote=self.NoteCam.getNumber("ty",0)

        self.txATag=self.ATagCam.getNumber("tx",0)
        self.tyATag=self.ATagCam.getNumber("ty",0)

        if ATagCamTargetSeen > 0:
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

        self.trapServo.set(constants.ServoClosed)

        self.driveTrain.AutoInit()

        autoname = pathPlanner.selectAuto(self.smartdashboard)
        self.automode = PathPlannerAuto(autoname)
        self.automode.schedule()

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        # This function is fairly empty because we're using the command scheduler to run our autonomous
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

        self.aTagPitch()
        
        speakerspeed = self.smartdashboard.getNumber("speakerspeed",shooter.Shooter.speakerscale)
        ampspeed = self.smartdashboard.getNumber("ampspeed",shooter.Shooter.ampscale)
        trapspeed = self.smartdashboard.getNumber("trapspeed",shooter.Shooter.trapscale)
        if self.controls.shootspeaker():
            self.shooter.fire(speakerspeed, self.controls.override(), self.controls.reverseOverride()) 
        elif self.controls.shootamp():
            self.shooter.fire(ampspeed, self.controls.override(), self.controls.reverseOverride())
        elif self.controls.shootTrap():
            self.shooter.fire(trapspeed, self.controls.override(), self.controls.reverseOverride())
        else: 
            self.shooter.stop()

        if self.controls.push_trap():
            if self.trapServo.get() < 0.15:
                self.trapServo.set(constants.ServoClosed)
            else:
                self.trapServo.set(constants.ServoOpen)

        
        forward = self.controls.forward() * self.getInputSpeed(1)
        horizontal = self.controls.horizontal() * self.getInputSpeed(1)
        rotate = self.controls.rotate() * self.getInputSpeed(1)
        pose = wpimath.geometry.Pose2d(forward, horizontal, rotate)
        fieldRelative = True
        
        # Overwrite movement from camera if we say so
        if self.controls.note_pickup():
            pose = self.noteLineup()
            self.shooter.fire(0, False, False)
            fieldRelative = False
        elif self.controls.target_amp():
            pose = self.line_up_to_target(self.kAmptags)
            fieldRelative = False
        elif self.controls.target_subwoofer():
            pose = self.line_up_to_target(self.kSpeakerTags)
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
        
        if self.controls.robotView():
            fieldRelative = False

        if self.controls.target_lock():
            pose = self.target_lock(pose)

        self.smartdashboard.putNumber("DriveX", pose.X())
        self.smartdashboard.putNumber("DriveY", pose.Y())
        self.smartdashboard.putNumber("DriveR", pose.rotation().degrees())

        self.driveTrain.Drive(pose, fieldRelative)

        self.climber.set(phoenix5.TalonFXControlMode.PercentOutput, self.controls.climber())

        if self.timer.hasElapsed(0.5):
            self.timer.reset()

    def aTagPitch(self):
        tags = self.get_target_list()
        tag = None
        if len(tags) == 0:
            return
        elif len(tags) == 1:
            tag = tags[0]
            
        distance = tag["t6t_rs"][2]

        height = wpimath.units.inchesToMeters(constants.speakerHeight)
        angle = math.atan2(height, distance)
        self.pitchServo.setAngle(angle * constants.servoConversion)

    def MoveToPose2d(self, pose: wpimath.geometry.Pose2d):
        trajectory = pose.relativeTo(self.lastOdometryPose)
        rotation = pose.rotation().radians() - self.lastOdometryPose.rotation().radians()

        self.smartdashboard.putNumber("trajectoryX", trajectory.X())
        self.smartdashboard.putNumber("trajectoryY", trajectory.Y())
        self.smartdashboard.putNumber("trajectoryR", rotation)

        rotate = rotation
        forward = trajectory.X()
        horizontal = trajectory.Y()

        if abs(forward) < 0.01:
            forward = 0.0
        if abs(horizontal) < 0.01:
            horizontal = 0.0
        if abs(rotate) < 0.01:
            rotate = 0.0

        output = wpimath.geometry.Pose2d(forward, horizontal, rotate)
        return output
    
    def line_up_to_target(self, tag_list):
        tList = self.get_target_list()

        # Exit early if we didn't find a tag or the tag is not one we were looking for.
        tid = self.filter_target_list(tList, tag_list)
        if not tid:
            return wpimath.geometry.Pose2d()
        
        # Target in robotspace Z value is the distance away, X is left/right, Y is up/down
        # 6 array of doubles: [x, y, z, rx, ry, rz]
        targetpose = tid["t6t_rs"]

        # Exit early if we didn't get a pose.
        if targetpose is None or len(targetpose) != 6:
            return wpimath.geometry.Pose2d()

        distance = targetpose[2]

        rotpid = 0.15
        rot = tid["tx"] * rotpid * -1

        rotation = wpimath.geometry.Rotation2d.fromDegrees(rot)

        # Leave some offset away from the speaker tags.
        #TODO also special case the pids for the speaker
        if tid["fID"] in self.kSpeakerTags:
            distance -= 2.1

        pid = 0.05
        fwd = distance * pid * -1
        return wpimath.geometry.Pose2d(fwd,0,rotation)
    
    def target_lock(self, pose: wpimath.geometry.Pose2d):
        '''target_lock will make us rotate to face the speaker tags so that we can continue moving while shooting.'''
        output = pose

        tList = self.get_target_list()

        tid = self.filter_target_list(tList, self.kSpeakerCenterTags)
        if not tid:
            return output
        
        rotpid = 0.19
        rot = tid["tx"] * rotpid * -1

        rotation = wpimath.geometry.Rotation2d.fromDegrees(rot)

        return wpimath.geometry.Pose2d(pose.X(), pose.Y(), rotation)


    def get_target_list(self):
        # TODO parse json to find multiple IDs in sight
        jsonBlob = self.ATagCam.getString("json", "{}")
        structuredBlob = json.loads(jsonBlob)
        if "Results" not in structuredBlob.keys():
            return []
        idList = structuredBlob["Results"]["Fiducial"]
        return idList
    
    def filter_target_list(self, foundList, searchList):
        for id in foundList:
            if id["fID"] in searchList:
                return id
        return None


    def get_target_pose(self):
        targetpose = self.ATagCam.getString("json", "{}")
        if len(targetpose) < 2:
            return []
        return targetpose

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
        '''goalX = math.tan(self.tyNote + self.robotToNoteCamera.rotation().y_degrees)*self.robotToNoteCamera.Z()
        goalX = 0.0 # TODO testing rotations only so we don't kill anyone.
        goalY = 0.0
        goalRotation = -self.txNote * 0.01 # TODO this is really slow to debug tracking notes. 0.1 worked ok.
        rotation = wpimath.geometry.Rotation2d.fromDegrees(goalRotation)
        goal = wpimath.geometry.Pose2d(goalX, goalY, rotation)
        return goal'''

        if abs(self.tyNote) > 0.001:
            pid = 0.05
            fwd = pid

            rotpid = 0.12
            rot = self.txNote * rotpid
            rot *= -1

            rotation = wpimath.geometry.Rotation2d.fromDegrees(rot)
            return wpimath.geometry.Pose2d(fwd,0,rotation)            
        return wpimath.geometry.Pose2d()
    
    def _simulationInit(self):
        self.driveTrain.SimInit()
    
    def _simulationPeriodic(self):
        self.driveTrain.SimUpdate(self.controls.rotate())

if __name__ == "__main__":
    wpilib.run(MyRobot)
    
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
