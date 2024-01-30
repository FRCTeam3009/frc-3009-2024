from wpimath.kinematics import SwerveDrive4Kinematics, SwerveDrive4Odometry
from wpimath.geometry import Translation2d
import swerve_module
import wpimath
import wpimath.geometry

class Chassis(object):
    def __init__(self):
        self._wheel_base_width = 21
        self._wheel_base_length = 25

class DriveTrain(object):
    def __init__(self, fl, fr, rl, rr):
        self.fl = swerve_module.SwerveModule(fl)
        self.fr = swerve_module.SwerveModule(fr)
        self.rl = swerve_module.SwerveModule(rl)
        self.rr = swerve_module.SwerveModule(rr)

        self.fl.reset_encoders()
        self.fr.reset_encoders()
        self.rl.reset_encoders()
        self.rr.reset_encoders()

        self.fr._drive_motor.setInverted(True)
        self.rr._drive_motor.setInverted(True)

        self._chassis = Chassis()
        self._drive_kinematics = SwerveDrive4Kinematics(
            Translation2d(self._chassis._wheel_base_width / 2.0, self._chassis._wheel_base_length / 2.0),
            Translation2d( self._chassis._wheel_base_width/ 2.0, - self._chassis._wheel_base_length/ 2.0),
            Translation2d(-self._chassis._wheel_base_width / 2.0, self._chassis._wheel_base_length / 2.0),
            Translation2d(-self._chassis._wheel_base_width/ 2.0, -self._chassis._wheel_base_length / 2.0),
        )

        zeroRotate = wpimath.geometry.Rotation2d()
        self.odometry = SwerveDrive4Odometry(self._drive_kinematics, zeroRotate, self.getSwerveModulePositions())

    def Drive(self, chassisSpeeds, period):
        discretized = wpimath.kinematics.ChassisSpeeds.discretize(chassisSpeeds, period)
        fl, fr, rl, rr = self._drive_kinematics.toSwerveModuleStates(discretized)

        self.fl.set_swerve_state(fl)
        self.fr.set_swerve_state(fr)
        self.rl.set_swerve_state(rl)
        self.rr.set_swerve_state(rr)

    def getSwerveModulePositions(self):
        return (self.fl.getSwerveModulePosition(), self.fr.getSwerveModulePosition(), self.rl.getSwerveModulePosition(), self.rr.getSwerveModulePosition())
