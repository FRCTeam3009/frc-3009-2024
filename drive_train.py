from wpimath.kinematics import SwerveDrive4Kinematics, SwerveDrive4Odometry
from wpimath.geometry import Translation2d
import swerve_module
import wpimath
import wpimath.geometry
import wpimath.kinematics
from pathplannerlib.auto import AutoBuilder, PathPlannerAuto
from pathplannerlib.config import HolonomicPathFollowerConfig, ReplanningConfig, PIDConstants
from wpilib import DriverStation
from swerve_drive_params import SwerveDriveParams
import chassis
import wpilib


class DriveTrain(object):
    def __init__(self, chassis : chassis.Chassis, fl : SwerveDriveParams, fr : SwerveDriveParams, rl : SwerveDriveParams, rr : SwerveDriveParams, period, gyro):
        self._chassis = chassis

        self.fl = swerve_module.SwerveModule(fl, self._chassis)
        self.fr = swerve_module.SwerveModule(fr, self._chassis)
        self.rl = swerve_module.SwerveModule(rl, self._chassis)
        self.rr = swerve_module.SwerveModule(rr, self._chassis)
        self.period = period 
        self.kMaxSpeed = 4.0 # meters per second
        self.kMaxRotate = self.kMaxSpeed / self._chassis._turn_meters_per_radian # radians per second4
        self.gyro = gyro
    

        self.fl.reset_encoders()
        self.fr.reset_encoders()
        self.rl.reset_encoders()
        self.rr.reset_encoders()

        self.fr._drive_motor.setInverted(True)
        self.rr._drive_motor.setInverted(True)

        self._drive_kinematics = SwerveDrive4Kinematics(
            Translation2d( self._chassis._wheel_base_width / 2.0, self._chassis._wheel_base_length / 2.0),
            Translation2d( self._chassis._wheel_base_width / 2.0, -self._chassis._wheel_base_length / 2.0),
            Translation2d(-self._chassis._wheel_base_width / 2.0, self._chassis._wheel_base_length / 2.0),
            Translation2d(-self._chassis._wheel_base_width / 2.0, -self._chassis._wheel_base_length / 2.0),
        )

        zeroRotate = wpimath.geometry.Rotation2d()
        self.odometry = SwerveDrive4Odometry(self._drive_kinematics, zeroRotate, self.getSwerveModulePositions())

        AutoBuilder.configureHolonomic(
            self.odometry.getPose, # Robot pose supplier
            self.odometry.resetPosition, # Method to reset odometry (will be called if your auto has a starting pose)
            self.getspeeds, # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            self.Drive, # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            HolonomicPathFollowerConfig( # HolonomicPathFollowerConfig, this should likely live in your Constants class
                PIDConstants(fl._drive_motor.pid_p, fl._drive_motor.pid_i, fl._drive_motor.pid_d), # Translation PID constants
                PIDConstants(fl._angle_motor.pid_p, fl._angle_motor.pid_i, fl._angle_motor.pid_d), # Rotation PID constants
                4.0, # Max module speed, in m/s
                chassis._turn_circumference, # Drive base radius in meters. Distance from robot center to furthest module.
                ReplanningConfig() # Default path replanning config. See the API for the options here
            ),
            self.shouldFlipPath, # Supplier to control path flipping based on alliance color
            self # Reference to this subsystem to set requirements
        )


    def Drive(self, chassisSpeeds):
        discretized = wpimath.kinematics.ChassisSpeeds.discretize(chassisSpeeds, self.period)
        swerve_states = self._drive_kinematics.toSwerveModuleStates(discretized)
        
        fl, fr, rl, rr = wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(swerve_states, self.kMaxSpeed)

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
    def resetPosition(self, pose:wpimath.geometry.Pose2d):
        angle = self.gyro.getAngle(wpilib.ADIS16470_IMU.IMUAxis.kYaw)
        self.odometry.resetPosition(angle, self.getSwerveModulePositions(), pose)