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
import pathPlanner

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

        self.NoteCam = self.nt.getTable("limelight-front")
        self.ATagCam = self.nt.getTable("limelight-back")

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

        pathplannerlib.auto.NamedCommands.registerCommand("shootSpeaker", pathPlanner.shootCommand(self.shooter, shooter.Shooter.speakerscale))
        pathplannerlib.auto.NamedCommands.registerCommand("shootAmp", pathPlanner.shootCommand(self.shooter, shooter.Shooter.ampscale))
        pathplannerlib.auto.NamedCommands.registerCommand("targetSpeaker", pathPlanner.lineAprilCommand(self.driveTrain, self.line_up_to_target, self.kSubwoofertags))
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

        self.autoName = ""

        self.selectAuto()

        

    def robotPeriodic(self):

        commands2.CommandScheduler.getInstance().run()

        swerveModulePositions = self.driveTrain.getSwerveModulePositions()
        rotation = wpimath.geometry.Rotation2d.fromDegrees(self.GetRotation())

        self.automode = None

        self.lastOdometryPose = self.driveTrain.odometry.update(rotation, swerveModulePositions)
        
        self.smartdashboard.putNumber("autoMode", self.autoToggle.get())

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
        
        self.smartdashboard.putBoolean("hasNote", self.shooter.hasNote())

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

        #autoMode = self.autoChooser.getSelected()
        autoMode= "WCSMiddleBlue"

        self.trapServo.set(constants.ServoClosed)

        '''autoName = "WCS"
        autoColor = "Red"'''

        self.driveTrain.AutoInit()
        '''if self.autoToggle.get():
            self.automode = PathPlannerAuto(autoName + "Right" + autoColor)
        else:
            self.automode = PathPlannerAuto(autoName + "Left" + autoColor)'''

        #TODO: Figure out if we want to use this or the switch
        self.automode = PathPlannerAuto(self.autoName)
        self.automode.schedule()

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        # This function is fairly empty because we're using the command scheduler to run our autonomous
        self.climber.set(phoenix5.TalonFXControlMode.PercentOutput, 0)

    def selectAuto(self):
        autoMode = self.smartdashboard.getNumber("autoMode", 0)
        if not autoMode in pathPlanner.autoMap:
            autoMode = 0
        self.autoName = pathPlanner.autoMap[autoMode]

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
        elif self.controls.shootTrap():
            self.shooter.fire(shooter.Shooter.trapscale, self.controls.override(), self.controls.reverseOverride())
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
    
    def GetRotation(self):
        return self.driveTrain.GetRotation()
    
    def line_up_to_target(self, tag_list):
        tid = self.ATagCam.getEntry("tid").getDoubleArray(None)
        tagFound = tid is not None and len(tid) > 0
        if tagFound and tid[0] in tag_list:
            '''
            r = -self.txATag
            verticalAngle = self.tyATag - self.robotToAprilCamera.rotation().y_degrees
            x = -math.tan(verticalAngle) * self.robotToAprilCamera.Z()
            # if we're targeting the speaker adjust for the base
            if tid[0] in self.kSubwoofertags:
                x = x - 1.2
            return wpimath.geometry.Pose2d(x,0,r)
            '''
            pid = 0.1
            fwd = self.tyATag * pid

            rotpid = 0.02
            rot = self.txATag * rotpid
            rot *= -1

            if tid[0] in self.kSubwoofertags:
                fwd = fwd - 1.2
            return wpimath.geometry.Pose2d(fwd,0,rot)            
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
        '''goalX = math.tan(self.tyNote + self.robotToNoteCamera.rotation().y_degrees)*self.robotToNoteCamera.Z()
        goalX = 0.0 # TODO testing rotations only so we don't kill anyone.
        goalY = 0.0
        goalRotation = -self.txNote * 0.01 # TODO this is really slow to debug tracking notes. 0.1 worked ok.
        rotation = wpimath.geometry.Rotation2d.fromDegrees(goalRotation)
        goal = wpimath.geometry.Pose2d(goalX, goalY, rotation)
        return goal'''

        if abs(self.tyNote) > 0.001:
            pid = 0.1
            fwd = self.tyNote * pid

            rotpid = 0.02
            rot = self.txNote * rotpid
            rot *= -1

            return wpimath.geometry.Pose2d(fwd,0,rot)            
        return wpimath.geometry.Pose2d()
    
    def _simulationPeriodic(self):
        testAngle = 180
        self.driveTrain.gyroSim.setGyroAngleY(testAngle)
        self.driveTrain.gyro.setGyroAngleY(testAngle)
        self.driveTrain.simUpdate()

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
