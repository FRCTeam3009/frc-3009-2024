import rev
import phoenix6
import math
from swerve_drive_params import SwerveDriveParams as sdp
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from wpimath.controller import SimpleMotorFeedforwardMeters
import wpilib
import wpimath.units
import chassis
import neoMotor
import krakenMotor
import constants

class SwerveModule(object):
    def __init__(self, sdp_: sdp, chassis_: chassis.Chassis):
        self._sdp = sdp_
        self._chassis = chassis_
        driveInverted = self.is_drive_inverted(self._sdp)
        #self._drive_module = neoMotor.neoMotor(self._sdp._drive_motor.id, False, self._chassis._driveMotorConversionFactor)
        self._drive_module = krakenMotor.krakenMotor(self._sdp._drive_motor.id, driveInverted, self._chassis._driveMotorConversionFactor)
        self._angle_module = neoMotor.neoMotor(self._sdp._angle_motor.id, True, self._chassis._angleMotorConversionFactor)

        self._encoder = phoenix6.hardware.CANcoder(self._sdp._angle_encoder.id)
        encoder_config = phoenix6.configs.CANcoderConfiguration()
        encoder_config.magnet_sensor.magnet_offset = self._sdp._angle_encoder.offset * -1
        self._encoder.configurator.apply(encoder_config)
        self._encoder.get_position().set_update_frequency(100)
        self._encoder.get_absolute_position().set_update_frequency(100)
        self._encoder.get_velocity().set_update_frequency(100)

        self._angle_module.setupPid(self._sdp._angle_motor.pid_p, self._sdp._angle_motor.pid_i, self._sdp._angle_motor.pid_d,
        0, 0.00000004, True, -1.0, 1.0, -math.pi, math.pi)

        #self._drive_module.setupPid(self._sdp._drive_motor.pid_p, self._sdp._drive_motor.pid_i, self._sdp._drive_motor.pid_d,
        #0, 0.00000005, False, -1.0, 1.0, False, False)
        self._drive_module.setupPid(self._sdp._drive_motor.pid_p, self._sdp._drive_motor.pid_i, self._sdp._drive_motor.pid_d,
        self._chassis._k_s, self._chassis._k_a, self._chassis._k_v)

        self.speedRPM = 0.0
        self.driveFF = 0.0

        self.timer = wpilib.Timer()
        self.timer.start()

    def is_drive_inverted(self, params : sdp):
        if constants.is_simulation:
            return False
        
        i = params._drive_motor.id
        if i == constants.FLDrive or i == constants.RLDrive:
            return True
        return False

    def get_drive_position(self):
        return self._drive_module.getPosition()
    
    def get_angle_position(self):
        return self._angle_module.getPosition()
    
    def get_drive_velocity(self):
        return self._drive_module.getVelocity()
    
    def get_angle_velocity(self):
        return self._angle_module.getVelocity()
    
    def get_angle_absolute(self):
        posStatus = self._encoder.get_absolute_position()
        rotations = posStatus.value_as_double
        rads = wpimath.units.rotationsToRadians(rotations)
        return rads
    
    def reset_encoders(self):
        self._drive_module.setPosition(0)
        self._angle_module.setPosition(self.get_angle_absolute())
    
    def get_swerve_state(self):
        rotation = Rotation2d(self.get_angle_absolute())
        return SwerveModuleState(self.get_drive_velocity(), rotation)

    def set_swerve_state(self, swerve_module_state_: SwerveModuleState):
        if abs(swerve_module_state_.speed) < 0.001:
            self.stop()
            return

        swerve_module_state_ = SwerveModuleState.optimize(swerve_module_state_, self.get_swerve_state().angle)
        self._angle_module.setReference(swerve_module_state_.angle.radians(), rev.CANSparkMax.ControlType.kPosition)

        self.speedRPM = swerve_module_state_.speed * 60 / self._chassis._driveMotorConversionFactor
        self.driveFF = 0.0000005
        #self._drive_module.setReference(self.speedRPM, rev.CANSparkMax.ControlType.kVelocity, arbFF = self.driveFF)
        self._drive_module.setReference(self.speedRPM, self.driveFF)

    def stop(self):
        self._drive_module.set(0)
        self._angle_module.set(0)

    def getSwerveModulePosition(self):
        distance = self.get_drive_position()
        rotation = Rotation2d(self.get_angle_absolute())
        return SwerveModulePosition(distance, rotation)
