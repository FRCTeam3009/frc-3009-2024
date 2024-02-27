from wpimath.kinematics import SwerveDrive4Kinematics, SwerveDrive4Odometry
from wpimath.geometry import Translation2d
import swerve_module
import wpimath
import wpimath.geometry
import wpimath.kinematics
import pathplannerlib.auto
from pathplannerlib.config import HolonomicPathFollowerConfig, ReplanningConfig, PIDConstants
from swerve_drive_params import SwerveDriveParams
import chassis
import wpilib
import wpilib.simulation

class DriveTrain():
    def __init__(self, chassis : chassis.Chassis, fl : SwerveDriveParams, fr : SwerveDriveParams, rl : SwerveDriveParams, rr : SwerveDriveParams, period):
        self._chassis = chassis

        self.fl = swerve_module.SwerveModule(fl, self._chassis)
        self.fr = swerve_module.SwerveModule(fr, self._chassis)
        self.rl = swerve_module.SwerveModule(rl, self._chassis)
        self.rr = swerve_module.SwerveModule(rr, self._chassis)
        self.period = period 
        self.maxSpeed = 4.0 # meters per second
        self.maxRotate = self.maxSpeed / self._chassis._turn_meters_per_radian # radians per second4
        self.gyro = wpilib.ADIS16470_IMU()
        self.gyroSim = wpilib.simulation.ADIS16470_IMUSim(self.gyro)

        self.fl.reset_encoders()
        self.fr.reset_encoders()
        self.rl.reset_encoders()
        self.rr.reset_encoders()

        self.fl._drive_motor.setInverted(True)
        self.rl._drive_motor.setInverted(True)

        self._drive_kinematics = SwerveDrive4Kinematics(
            Translation2d( self._chassis._wheel_base_width / 2.0, self._chassis._wheel_base_length / 2.0),
            Translation2d( self._chassis._wheel_base_width / 2.0, -self._chassis._wheel_base_length / 2.0),
            Translation2d(-self._chassis._wheel_base_width / 2.0, self._chassis._wheel_base_length / 2.0),
            Translation2d(-self._chassis._wheel_base_width / 2.0, -self._chassis._wheel_base_length / 2.0),
        )

        zeroRotate = wpimath.geometry.Rotation2d()
        self.odometry = SwerveDrive4Odometry(self._drive_kinematics, zeroRotate, self.getSwerveModulePositions())

    def AutoInit(self):
        driveP = self.fl._drive_pid_controller.getP()
        driveI = self.fl._drive_pid_controller.getI()
        driveD = self.fl._drive_pid_controller.getD()

        angleP = self.fl._drive_pid_controller.getP()
        angleI = self.fl._drive_pid_controller.getI()
        angleD = self.fl._drive_pid_controller.getD()

        if not pathplannerlib.auto.AutoBuilder.isConfigured():
            pathplannerlib.auto.AutoBuilder.configureHolonomic(
                self.odometry.getPose, # Robot pose supplier
                self.resetPosition, # Method to reset odometry (will be called if your auto has a starting pose)
                self.getspeeds, # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                self.DriveRobotRelative, # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                HolonomicPathFollowerConfig( # HolonomicPathFollowerConfig, this should likely live in your Constants class
                    PIDConstants(driveP, driveI, driveD), # Translation PID constants
                    PIDConstants(angleP, angleI, angleD), # Rotation PID constants
                    4.0, # Max module speed, in m/s
                    self._chassis._turn_circumference, # Drive base radius in meters. Distance from robot center to furthest module.
                    ReplanningConfig() # Default path replanning config. See the API for the options here
                ),
                self.shouldFlipPath, # Supplier to control path flipping based on alliance color
                None # Reference to this subsystem to set requirements
            )

    def Drive(self, pose: wpimath.geometry.Pose2d, fieldRelative):
         # Cap the speeds
        forward = capValue(pose.X(), self.maxSpeed)
        horizontal = capValue(pose.Y(), self.maxSpeed)
        rotate = capValue(pose.rotation().radians(), self.maxRotate)

        gyroYaw = self.GetRotation()
        relativeRotation = wpimath.geometry.Rotation2d.fromDegrees(gyroYaw)

        chassisSpeeds = wpimath.kinematics.ChassisSpeeds.fromRobotRelativeSpeeds(forward, horizontal, rotate, relativeRotation)
        if fieldRelative:
            chassisSpeeds = wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(forward, horizontal, rotate, relativeRotation)

        self.chassisSpeeds = wpimath.kinematics.ChassisSpeeds.discretize(chassisSpeeds, self.period)
        self.DriveRobotRelative(chassisSpeeds)

    def DriveRobotRelative(self, chassisSpeeds):
        discretized = wpimath.kinematics.ChassisSpeeds.discretize(chassisSpeeds, self.period)
        swerve_states = self._drive_kinematics.toSwerveModuleStates(discretized)
        
        fl, fr, rl, rr = wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(swerve_states, self.maxSpeed)

        self.fl.set_swerve_state(fl)
        self.fr.set_swerve_state(fr)
        self.rl.set_swerve_state(rl)
        self.rr.set_swerve_state(rr)

    def getSwerveModulePositions(self):
        return (self.fl.getSwerveModulePosition(), self.fr.getSwerveModulePosition(), self.rl.getSwerveModulePosition(), self.rr.getSwerveModulePosition())

    def shouldFlipPath(self):
        return False
    
    def getspeeds(self):
        swerevestates = [self.fl.get_swerve_state(), self.fr.get_swerve_state(), self.rl.get_swerve_state(), self.rr.get_swerve_state()]
        return self._drive_kinematics.toChassisSpeeds(swerevestates)
    
    def resetPosition(self, pose: wpimath.geometry.Pose2d):
        angle = self.GetRotation()
        rotate = wpimath.geometry.Rotation2d.fromDegrees(angle)
        self.odometry.resetPosition(rotate, self.getSwerveModulePositions(), pose)

    def GetRotation(self):
        return self.gyro.getAngle(wpilib.ADIS16470_IMU.IMUAxis.kPitch) * -1
    
    def UpdateMaxSpeed(self, speed):
        self.maxSpeed = speed
        self.maxRotate = self.maxSpeed / self._chassis._turn_meters_per_radian
    
    def simUpdate(self):
        self.fl.simUpdate(self.period)
        self.fr.simUpdate(self.period)
        self.rl.simUpdate(self.period)
        self.rr.simUpdate(self.period)

def capValue(value, cap):
    if value > cap:
        return cap
    elif value < -1 * cap:
        return -1 * cap
    else:
        return value
