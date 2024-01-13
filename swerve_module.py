import rev
import phoenix5
import phoenix5.sensors
import math
from swerve_drive_params import SwerveDriveParams as sdp
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
from wpimath.controller import SimpleMotorFeedforwardMeters


class SwerveModule(object):
    def __init__(self, sdp_):
        self._sdp = sdp_
        self._drive_motor = rev.CANSparkMax(sdp_._drive_motor['id'], rev.CANSparkLowLevel.MotorType.kBrushless)
        self._drive_motor.setInverted(False)
        self._angle_motor = rev.CANSparkMax(sdp_._angle_motor['id'], rev.CANSparkLowLevel.MotorType.kBrushless)

        self._drive_motor_encoder = self._drive_motor.getEncoder()
        self._drive_motor_encoder.setPositionConversionFactor((1/self._sdp._k_drive_gear_ratio) * math.pi * (self._sdp._k_wheel_diameter/39.37))
        self._drive_motor_encoder.setVelocityConversionFactor(self._drive_motor_encoder.getPositionConversionFactor() / 60)

        self._angle_motor_encoder = self._angle_motor.getEncoder()
        self._angle_motor_encoder.setPositionConversionFactor((1/self._sdp._k_angle_gear_ratio) * math.pi * 2)
        self._angle_motor_encoder.setVelocityConversionFactor(self._angle_motor_encoder.getVelocityConversionFactor() / 60)

        self._encoder = phoenix5.sensors.CANCoder(sdp_._angle_encoder['id'])
        self._encoder_config = phoenix5.sensors.CANCoderConfiguration()
        self._encoder_config.initializationStrategy.BootToAbsolutePosition
        self._encoder_config.magnetOffsetDegrees = sdp_._angle_encoder['offset']
        self._encoder_config.sensorTimeBase = phoenix5.sensors.SensorTimeBase.PerSecond
        self._encoder_config.sensorCoefficient = 2 * math.pi / 4096.0
        self._encoder_config.unitString = "rad"
        self._encoder_config.absoluteSensorRange = phoenix5.sensors.AbsoluteSensorRange.Signed_PlusMinus180
        self._encoder.configAllSettings(self._encoder_config)

        self._last_angle = self._encoder.getAbsolutePosition()
        self._feed_forward_controller = SimpleMotorFeedforwardMeters(sdp_._k_s, sdp_._k_v, sdp_._k_a)

        self._angle_pid_controller = self._angle_motor.getPIDController()
        self._angle_pid_controller.setP(self._sdp._drive_motor['pid_p'])
        self._angle_pid_controller.setI(self._sdp._drive_motor['pid_i'])
        self._angle_pid_controller.setD(self._sdp._drive_motor['pid_d'])
        self._angle_pid_controller.setIZone(0)
        self._angle_pid_controller.setFF(0)
        # TODO: Do we need wrapping?
        self._angle_pid_controller.setPositionPIDWrappingEnabled(True)

        if sdp_._name in ['rf', 'rr']:
            self._drive_motor.setInverted(True)

    def get_drive_position(self):
        return self._drive_motor_encoder.getPosition()
    
    def get_angle_position(self):
        return self._angle_motor_encoder.getPosition()
    
    def get_drive_velocity(self):
        return self._drive_motor_encoder.getVelocity()
    
    def get_angle_velocity(self):
        return self._angle_motor_encoder.getVelocity()
    
    def get_angle_absolute(self):
        return self._encoder.getAbsolutePosition()
    
    def reset_encoders(self):
        self._drive_motor_encoder.setPosition(0)
        self._angle_motor_encoder.setPosition(self.get_angle_absolute())
    
    def get_swerve_state(self):
        return SwerveModuleState(self.get_drive_velocity(), Rotation2d(self.get_angle_position()))

    def set_swerve_state(self, swerve_module_state_):
        if abs(swerve_module_state_.speed) < 0.001:
            self.stop()
            return
        swerve_module_state_ = SwerveModuleState.optimize(swerve_module_state_, self.get_swerve_state().angle)
        self._drive_motor.set(self._feed_forward_controller.calculate(swerve_module_state_.speed))
        self._angle_pid_controller.setReference(swerve_module_state_.angle.radians(), rev.CANSparkMax.ControlType.kPosition)

    def stop(self):
        self._drive_motor.set(0)
        self._angle_motor.set(0)
