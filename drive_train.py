from wpimath.kinematics import SwerveDrive4Kinematics, SwerveDrive4Odometry
from wpimath.geometry import Translation2d
import swerve_module
import wpimath
import wpimath.geometry
import wpimath.kinematics

class DriveTrain(object):
    def __init__(self, chassis, fl, fr, rl, rr):
        self._chassis = chassis

        self.fl = swerve_module.SwerveModule(fl, self._chassis)
        self.fr = swerve_module.SwerveModule(fr, self._chassis)
        self.rl = swerve_module.SwerveModule(rl, self._chassis)
        self.rr = swerve_module.SwerveModule(rr, self._chassis)

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

    def Drive(self, chassisSpeeds, period, max_speed):
        discretized = wpimath.kinematics.ChassisSpeeds.discretize(chassisSpeeds, period)
        swerve_states = self._drive_kinematics.toSwerveModuleStates(discretized)
        
        fl, fr, rl, rr = wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(swerve_states, max_speed)

        self.fl.set_swerve_state(fl)
        self.fr.set_swerve_state(fr)
        self.rl.set_swerve_state(rl)
        self.rr.set_swerve_state(rr)

    def getSwerveModulePositions(self):
        return (self.fl.getSwerveModulePosition(), self.fr.getSwerveModulePosition(), self.rl.getSwerveModulePosition(), self.rr.getSwerveModulePosition())
