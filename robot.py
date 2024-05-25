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
        self.ledStrips.solid(led.kHighScalersYellow)
        self.alliance = wpilib.DriverStation.getAlliance() # This is too soon for FMS

        self.kSpeakerTags = [3, 4, 7, 8]
        self.kSpeakerCenterTags = [4, 7]
        self.kAmptags = [5, 6] 
        self.kStagetags = [11, 12, 13, 14, 15, 16]
        self.kSourcetags = [1, 2, 9, 10]
        self.kAlltags = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]
        self.kAmpAndStage = self.kAmptags + self.kStagetags
        self.kDefaultScoopScale = 0.5
        self.kDefaultMiddleRampScale = 1.0
        self.kSubwooferDistance = constants.subwooferDistance
        self.kSubwooferStopDistance = constants.subwooferDistanceOffset

        self.lastOdometryPose = wpimath.geometry.Pose2d()

        self.nt = ntcore.NetworkTableInstance.getDefault()
        self.nt.startServer()
        self.smartdashboard = self.nt.getTable("SmartDashboard")
        self.smartdashboard.putNumber("scoop_speed", self.kDefaultScoopScale)
        self.smartdashboard.putNumber("middle_ramp_speed", self.kDefaultMiddleRampScale)
        self.smartdashboard.putNumber("amp_distance", 0.5)

        self.smartdashboard.putNumber("servo open", constants.ServoOpen)
        self.smartdashboard.putNumber("servo closed", constants.ServoClosed)

        self.smartdashboard.putNumber("pitchShooter", constants.shooterPitch)
        self.smartdashboard.putNumber("speedShooter", constants.shooterSpeed)
        self.smartdashboard.putNumber("distance", 10000)

        pathPlanner.autonomousDropdown(self.smartdashboard)

        self.NoteCam = self.nt.getTable("limelight-front")
        self.ATagCam = self.nt.getTable("limelight-back")

        self.trapServo = wpilib.Servo(constants.Servo)
        self.trapServo.set(constants.ServoClosed)
        self.buddyServo = wpilib.Servo(constants.buddyServo)
        self.buddyServo.set(constants.buddyServoClosed)

        self.pitchServo = wpilib.Servo(constants.pitchServo)
        self.set_shooter_angle(constants.speakerAngle)

        self.potInput = wpilib.AnalogInput(0)
        #self.potInput.setAverageBits(2)
        self.shooterPot = wpilib.AnalogPotentiometer(self.potInput, constants.servoFullRange, -40) # device, servo range, pot at 0v location

        p_value = 6e-5
        i_value = 1e-6
        d_value = 0

        angle_p_value = 0.5
        
        # Front Left
        fldriveMotorParams = motorParams.Motorparams(GetCanId(constants.FLDrive), p_value, i_value, d_value)
        flangleMotorParams = motorParams.Motorparams(GetCanId(constants.FLAngle), angle_p_value, i_value)
        flEncoderParams = encoderParams.EncoderParams(GetCanId(constants.FLEncoder), constants.FLEncoderOffset)
        flParams = swerve_drive_params.SwerveDriveParams(fldriveMotorParams, flangleMotorParams, flEncoderParams)

        # Rear Left
        rldriveMotorParams = motorParams.Motorparams(GetCanId(constants.RLDrive), p_value, i_value, d_value)
        rlangleMotorParams = motorParams.Motorparams(GetCanId(constants.RLAngle), angle_p_value, i_value)
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
        self.autoStateTimer = wpilib.Timer()

        pathplannerlib.auto.NamedCommands.registerCommand("shootSpeaker", pathPlanner.shootCommand(self.shooter, self.getShooterSpeedSetAngles))
        pathplannerlib.auto.NamedCommands.registerCommand("shootAmp", pathPlanner.shootCommand(self.shooter, self.getAmpShooterSpeedAngles))
        pathplannerlib.auto.NamedCommands.registerCommand("targetSpeaker", pathPlanner.lineAprilCommand(self.driveTrain, self.line_up_to_target, self.kSpeakerCenterTags, self.aprilDistance, 2.92))
        pathplannerlib.auto.NamedCommands.registerCommand("targetAmp", pathPlanner.lineAprilCommand(self.driveTrain, self.line_up_to_target_exact, self.kAmptags, self.aprilDistance, 1.1))
        pathplannerlib.auto.NamedCommands.registerCommand("targetClosest", pathPlanner.lineAprilCommand(self.driveTrain, self.line_up_to_target_exact, self.kAlltags, self.aprilDistance, 1.1))
        pathplannerlib.auto.NamedCommands.registerCommand("targetNote", pathPlanner.lineNoteCommand(self.driveTrain, self.noteLineup, self.shooter, self.noteFound))
        pathplannerlib.auto.NamedCommands.registerCommand("driveAway", pathPlanner.driveAwayCommand(self.driveTrain))

        self.txATag=0
        self.tyATag=0
        self.txNote=0
        self.tyNote=0
        self.botpose=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.startPose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.startPoselist = []
        self.startPoseCalibrating = True

        self.hadNote = False
        self.wasStopped = False

        self.autoFail = None

        self.distance = 0
        self.NoteCamTargetSeen = 0

    def robotPeriodic(self):
        self.alliance = wpilib.DriverStation.getAlliance()

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
        
        self.smartdashboard.putNumber("speakerspeed",shooter.Shooter.speakerspeed_close)
        self.smartdashboard.putNumber("ampspeed",shooter.Shooter.ampspeed)
        self.smartdashboard.putNumber("trapspeed",shooter.Shooter.trapspeed)

        self.smartdashboard.putBoolean("hasNote", self.shooter.hasNote())
        self.smartdashboard.putNumber("autoMode", 0)


        self.driveTrain.publishDashboardStates(self.smartdashboard)

        constants.ServoOpen = self.smartdashboard.getNumber("servo open", constants.ServoOpen)
        constants.ServoClosed = self.smartdashboard.getNumber("servo closed", constants.ServoClosed)

        constants.shooterPitch = self.smartdashboard.getNumber("pitchShooter", constants.shooterPitch)
        constants.shooterSpeed = self.smartdashboard.getNumber("speedShooter", constants.shooterSpeed)

        self.smartdashboard.putNumberArray("startpose", self.startPose)

        ATagCamTargetSeen = self.ATagCam.getNumber("tv",0)
        self.NoteCamTargetSeen = self.NoteCam.getNumber("tv",0)
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
    
        tags = self.get_target_list()
        tag = self.filter_target_list(tags, self.kSpeakerCenterTags)
        if tag is not None:
            self.distance = tag["t6t_rs"][2]  
        self.smartdashboard.putNumber("distance", self.distance)  

    def disabledInit(self):
        self.ledStrips.solid(led.kHighScalersYellow)

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        self.timer.reset()
        self.timer.start()

        self.autoStateTimer.reset()
        self.autoStateTimer.start()

        self.set_led_side()

        self.trapServo.set(constants.ServoClosed)
        self.buddyServo.set(constants.buddyServoClosed)

        self.driveTrain.AutoInit()

        #autoname = pathPlanner.selectAuto(self.smartdashboard)
        autoname = "stayAuto"
        self.automode = PathPlannerAuto(autoname)
        self.automode.schedule()

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        # This function is fairly empty because we're using the command scheduler to run our autonomous
        self.climber.set(phoenix5.TalonFXControlMode.PercentOutput, 0)

        zeroSpeed = speedIsZero(self.driveTrain.chassisSpeeds)
        if self.hadNote != self.shooter.hasNote():
            self.autoStateTimer.reset()
            self.hadNote = self.shooter.hasNote()
        if zeroSpeed != self.wasStopped:
            self.autoStateTimer.reset()
            self.wasStopped = zeroSpeed
        if self.autoStateTimer.hasElapsed(10):
            pass
            # commands2.CommandScheduler.getInstance().cancelAll()
            # if self.autoFail is None:
            #     self.autoFail = self.driveTrain.odometry.getPose()
            # currentPose = self.driveTrain.odometry.getPose()
            # default drive backwards
            # speed = 1
            # if abs(self.autoFail.X() - currentPose.X()) >= constants.autoDefaultDistance:
            #     speed = 0
            # csp = wpimath.kinematics.ChassisSpeeds(speed, 0, 0)
            # self.driveTrain.DriveRobotRelative(csp)
            # TODO make better system

    def teleopInit(self):
        """This function is run once each time the robot enters teleop mode."""
        self.driveTrain.UpdateMaxSpeed(constants.MaxSpeed)
        self.trapServo.set(constants.ServoClosed)
        self.buddyServo.set(constants.buddyServoClosed)

        commands2.CommandScheduler.getInstance().cancelAll()
        self.timer.reset()
        self.timer.start()

        self.set_led_side()

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        if self.controls.reset_gyro():
            self.driveTrain.gyro.reset()

        speakerspeed = 0
        ampspeed = self.smartdashboard.getNumber("ampspeed",shooter.Shooter.ampspeed)
        trapspeed = self.smartdashboard.getNumber("trapspeed",shooter.Shooter.trapspeed)

        if self.controls.ampPitch():
            self.set_shooter_angle(constants.ampAngle) # Assuming trap and amp are same angle
        elif self.controls.speakerPitch():
            self.set_shooter_angle(constants.speakerAngle)
            speakerspeed = shooter.Shooter.speakerspeed_far
            # TODO set this to the correct pitch which is the first in the dictionary
        elif self.controls.trapPitch():
            self.set_shooter_angle(constants.trapAngle)
        else:
            speakerspeed = self.getShooterSpeedSetAngles()

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
        
        if self.controls.buddyBar():
            average = (constants.buddyServoClosed + constants.buddyServoOpen) / 2
            if self.buddyServo.get() > average:
                self.buddyServo.set(constants.buddyServoOpen)
            else:
                self.buddyServo.set(constants.buddyServoClosed)

        
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
        elif self.controls.target_speaker():
            pose = self.line_up_to_target(self.kSpeakerCenterTags)
            magnitude = math.sqrt(pose.X()**2 + pose.Y()**2)
            if magnitude < self.kSubwooferStopDistance:
                pose = wpimath.geometry.Pose2d()
            fieldRelative = False
        elif self.controls.target_closest():
            pose = self.line_up_to_target_exact(self.kAmpAndStage)
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
        if tid["fID"] in self.kSpeakerCenterTags:
            distance -= 2.1

        pid = 0.05
        fwd = distance * pid * -1
        return wpimath.geometry.Pose2d(fwd,0,rotation)
    
    def line_up_to_target_exact(self, tag_list):
        '''This will do both forward and horizontal adjustments as well as rotation'''
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

        distForward = targetpose[2] # The Target's Z axis is outward from the camera.
        distHorz = targetpose[0] # The Target's X axis is left/right.

        rotpid = 0.15
        rot = tid["tx"] * rotpid * -1

        if abs(rot) < 1:
            rot = 0

        rotation = wpimath.geometry.Rotation2d.fromDegrees(rot)

        # Leave some offset away from the speaker tags.
        if tid["fID"] in self.kSpeakerCenterTags:
            distForward -= 2.1
        else:
            distForward -= 1.1

        pid = 0.05
        forward = distForward * pid * -1 # The camera's are facing backwards, so negative
        horizontal = distHorz * pid

        return wpimath.geometry.Pose2d(forward, horizontal, rotation)
    
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
            return 0.75 * speed
            #return 0.10 * speed
        elif self.controls.slow():
            return 0.25 * speed
            #return 0.10 * speed
        else:
            return 0.50 * speed
            #return 0.10 * speed
        

    def testInit(self) -> None:
        commands2.CommandScheduler.getInstance().cancelAll()

    def noteLineup(self):

        if abs(self.tyNote) > 0.001:
            pid = 0.075
            fwd = pid

            rotpid = 0.12
            rot = self.txNote * rotpid
            rot *= -1

            rotation = wpimath.geometry.Rotation2d.fromDegrees(rot)
            return wpimath.geometry.Pose2d(fwd,0,rotation)            
        return wpimath.geometry.Pose2d()
    
    def noteFound(self):
        return self.NoteCamTargetSeen == 1
    
    def set_shooter_angle(self, angle):
        if angle < constants.shooterAngleMin:
            angle = constants.shooterAngleMin
        if angle > constants.shooterAngleMax:
            angle = constants.shooterAngleMax

        a = convert_shooter_angle_to_servo_value(angle)
        self.pitchServo.set(a)

    def set_led_side(self):
        if self.alliance is None:
            self.ledStrips.solid(led.kGreen)
        elif self.alliance.kBlue:
            self.ledStrips.solid(led.kBumperBlue)
        else:
            self.ledStrips.solid(led.kBumperRed)
    
    def _simulationInit(self):
        self.driveTrain.SimInit()
    
    def _simulationPeriodic(self):
        self.driveTrain.SimUpdate(self.controls.rotate())
    
    def getShooterSpeedSetAngles(self):
        sa = shooter.lookUpAngleSpeed(self.distance)
        self.set_shooter_angle(sa.angle)
        return sa.speed
    
    def getAmpShooterSpeedAngles(self):
        self.set_shooter_angle(constants.ampAngle)
        return shooter.Shooter.ampspeed
    
    def aprilDistance(self):
        tList = self.get_target_list()

        # Exit early if we didn't find a tag or the tag is not one we were looking for.
        tid = self.filter_target_list(tList, self.kSpeakerCenterTags)
        if not tid:
            return -1
        
        # Target in robotspace Z value is the distance away, X is left/right, Y is up/down
        # 6 array of doubles: [x, y, z, rx, ry, rz]
        targetpose = tid["t6t_rs"]

        # Exit early if we didn't get a pose.
        if targetpose is None or len(targetpose) != 6:
            return -1

        return targetpose[2]


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

def convert_servo_angle_to_value(angle):
    servo_angle_range = constants.servoPotMax-constants.servoPotMin
    normalized_angle = (angle - constants.servoPotMin)/servo_angle_range
    normalized_angle = 1.0 - normalized_angle

    servo_value_range = constants.servoMaxValue-constants.servoMinValue
    return (normalized_angle*servo_value_range) + constants.servoMinValue


def convert_shooter_angle_to_servo_value(angle):
    shooter_angle_range = constants.shooterAngleMax-constants.shooterAngleMin
    normalized_angle = (angle - constants.shooterAngleMin)/shooter_angle_range
    normalized_angle = 1.0 - normalized_angle

    servo_value_range = constants.servoMaxValue-constants.servoMinValue
    return (normalized_angle*servo_value_range) + constants.servoMinValue

def speedIsZero(chassisSpeed: wpimath.kinematics.ChassisSpeeds):
    return chassisSpeed.vx == 0 and chassisSpeed.vy == 0 and chassisSpeed.omega == 0

def autoIsBlue(automode: str):
    return "blue" in automode.lower()