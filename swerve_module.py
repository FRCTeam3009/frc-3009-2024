import rev
import phoenix5
import phoenix5.sensors
import phoenix6
import math
from swerve_drive_params import SwerveDriveParams as sdp
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState
from wpimath.controller import SimpleMotorFeedforwardMeters
import wpilib
import wpimath.units

class SwerveModule(object):
    def __init__(self, sdp_: sdp):
        self._sdp = sdp_
        self._drive_motor = rev.CANSparkMax(sdp_._drive_motor.id, rev.CANSparkLowLevel.MotorType.kBrushless)
        self._drive_motor.setInverted(False)
        self._angle_motor = rev.CANSparkMax(sdp_._angle_motor.id, rev.CANSparkLowLevel.MotorType.kBrushless)
        self._angle_motor.setInverted(True)
        driveMotorConversionFactor = (1/self._sdp._k_drive_gear_ratio) * math.pi * (self._sdp._k_wheel_diameter/39.37)
        angleMotorConversionFactor = ((1/self._sdp._k_angle_gear_ratio) * math.pi * 2)

        self._drive_motor_encoder = self._drive_motor.getEncoder()
        self._drive_motor_encoder.setPositionConversionFactor(driveMotorConversionFactor)
        self._drive_motor_encoder.setVelocityConversionFactor(self._drive_motor_encoder.getPositionConversionFactor() / 60)

        self._angle_motor_encoder = self._angle_motor.getEncoder()
        self._angle_motor_encoder.setPositionConversionFactor(angleMotorConversionFactor)
        self._angle_motor_encoder.setVelocityConversionFactor(self._angle_motor_encoder.getVelocityConversionFactor() / 60)

        self._encoder = phoenix6.hardware.CANcoder(sdp_._angle_encoder.id)
        encoder_config = phoenix6.configs.CANcoderConfiguration()
        encoder_config.magnet_sensor.magnet_offset = sdp_._angle_encoder.offset * -1
        self._encoder.configurator.apply(encoder_config)
        self._encoder.get_position().set_update_frequency(100)
        self._encoder.get_absolute_position().set_update_frequency(100)
        self._encoder.get_velocity().set_update_frequency(100)

        self._feed_forward_controller = SimpleMotorFeedforwardMeters(sdp_._k_s, sdp_._k_v, sdp_._k_a)
        #TODO limit the angle controller
        self._angle_feed_forward_controller = SimpleMotorFeedforwardMeters(sdp_._k_s, sdp_._k_v, sdp_._k_a)

        self._angle_pid_controller = self._angle_motor.getPIDController()
        self._angle_pid_controller.setP(self._sdp._angle_motor.pid_p)
        self._angle_pid_controller.setI(self._sdp._angle_motor.pid_i)
        self._angle_pid_controller.setD(self._sdp._angle_motor.pid_d)
        self._angle_pid_controller.setIZone(0)
        self._angle_pid_controller.setFF(0)
        self._angle_pid_controller.setPositionPIDWrappingEnabled(True)
        self._angle_pid_controller.setOutputRange(-0.2, 0.2)
        self._angle_pid_controller.setPositionPIDWrappingMinInput(-math.pi)
        self._angle_pid_controller.setPositionPIDWrappingMaxInput(math.pi)

        self.timer = wpilib.Timer()
        self.timer.start()

    def get_drive_position(self):
        return self._drive_motor_encoder.getPosition()
    
    def get_angle_position(self):
        return self._angle_motor_encoder.getPosition()
    
    def get_drive_velocity(self):
        return self._drive_motor_encoder.getVelocity()
    
    def get_angle_velocity(self):
        return self._angle_motor_encoder.getVelocity()
    
    def get_angle_absolute(self):
        pos = self._encoder.get_absolute_position()
        rotations = pos.value_as_double
        rads = wpimath.units.rotationsToRadians(rotations)
        return rads
    
    def reset_encoders(self):
        self._drive_motor_encoder.setPosition(0)
        self._angle_motor_encoder.setPosition(self.get_angle_absolute())
    
    def get_swerve_state(self):
        rotation = Rotation2d(self.get_angle_position())
        return SwerveModuleState(self.get_drive_velocity(), rotation)

    def set_swerve_state(self, swerve_module_state_: SwerveModuleState):
        if abs(swerve_module_state_.speed) < 0.001:
            self.stop()
            return
        swerve_module_state_ = SwerveModuleState.optimize(swerve_module_state_, self.get_swerve_state().angle)
        self._drive_motor.set(self._feed_forward_controller.calculate(swerve_module_state_.speed))

        self._angle_pid_controller.setReference(swerve_module_state_.angle.radians(), rev.CANSparkMax.ControlType.kPosition)

    def stop(self):
        self._drive_motor.set(0)
        self._angle_motor.set(0)
