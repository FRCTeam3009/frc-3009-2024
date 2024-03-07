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

class SwerveModule(object):
    def __init__(self, sdp_: sdp, chassis_: chassis.Chassis):
        self._sdp = sdp_
        self._chassis = chassis_
        self._drive_motor = rev.CANSparkMax(self._sdp._drive_motor.id, rev.CANSparkLowLevel.MotorType.kBrushless)
        self._drive_motor.setInverted(False)
        self._angle_motor = rev.CANSparkMax(self._sdp._angle_motor.id, rev.CANSparkLowLevel.MotorType.kBrushless)
        self._angle_motor.setInverted(True)

        self._drive_motor_encoder = self._drive_motor.getEncoder()
        self._drive_motor_encoder.setPositionConversionFactor(self._chassis._driveMotorConversionFactor)
        self._drive_motor_encoder.setVelocityConversionFactor(self._drive_motor_encoder.getPositionConversionFactor())

        self._angle_motor_encoder = self._angle_motor.getEncoder()
        self._angle_motor_encoder.setPositionConversionFactor(self._chassis._angleMotorConversionFactor)
        self._angle_motor_encoder.setVelocityConversionFactor(self._angle_motor_encoder.getPositionConversionFactor())

        self._encoder = phoenix6.hardware.CANcoder(self._sdp._angle_encoder.id)
        encoder_config = phoenix6.configs.CANcoderConfiguration()
        encoder_config.magnet_sensor.magnet_offset = self._sdp._angle_encoder.offset * -1
        self._encoder.configurator.apply(encoder_config)
        self._encoder.get_position().set_update_frequency(100)
        self._encoder.get_absolute_position().set_update_frequency(100)
        self._encoder.get_velocity().set_update_frequency(100)

        self._angle_pid_controller = self._angle_motor.getPIDController()
        self._angle_pid_controller.setP(self._sdp._angle_motor.pid_p)
        self._angle_pid_controller.setI(self._sdp._angle_motor.pid_i)
        self._angle_pid_controller.setD(self._sdp._angle_motor.pid_d)
        self._angle_pid_controller.setIZone(0)
        self._angle_pid_controller.setFF(0.00000004)
        #self._angle_pid_controller.setFF(self._sdp._angle_k_v)
        self._angle_pid_controller.setPositionPIDWrappingEnabled(True)
        self._angle_pid_controller.setOutputRange(-1.0, 1.0)
        self._angle_pid_controller.setPositionPIDWrappingMinInput(-math.pi)
        self._angle_pid_controller.setPositionPIDWrappingMaxInput(math.pi)

        self._drive_pid_controller = self._drive_motor.getPIDController()
        self._drive_pid_controller.setP(self._sdp._drive_motor.pid_p)
        self._drive_pid_controller.setI(self._sdp._drive_motor.pid_i)
        self._drive_pid_controller.setD(self._sdp._drive_motor.pid_d)
        self._drive_pid_controller.setIZone(0)
        #self._drive_pid_controller.setFF(self._chassis._k_v)
        self._drive_pid_controller.setFF(0.00000005)
        self._drive_pid_controller.setOutputRange(-1.0, 1.0)

        self.drive_feed_forward = SimpleMotorFeedforwardMeters(self._chassis._k_s, self._chassis._k_v, self._chassis._k_a)

        self.speedRPM = 0.0
        self.driveFF = 0.0

        self.timer = wpilib.Timer()
        self.timer.start()

        self.simulation = False
        self.simAngle = 0

    def get_drive_position(self):
        return self._drive_motor_encoder.getPosition()
    
    def get_angle_position(self):
        return self._angle_motor_encoder.getPosition()
    
    def get_drive_velocity(self):
        return self._drive_motor_encoder.getVelocity()
    
    def get_angle_velocity(self):
        return self._angle_motor_encoder.getVelocity()
    
    def get_angle_absolute(self):
        posStatus = self._encoder.get_absolute_position()
        rotations = posStatus.value_as_double
        if self.simulation:
            rotations = self.get_angle_position()
        rads = wpimath.units.rotationsToRadians(rotations)
        return rads
    
    def reset_encoders(self):
        self._drive_motor_encoder.setPosition(0)
        self._angle_motor_encoder.setPosition(self.get_angle_absolute())
    
    def get_swerve_state(self):
        rotation = Rotation2d(self.get_angle_absolute())
        return SwerveModuleState(self.get_drive_velocity(), rotation)

    def set_swerve_state(self, swerve_module_state_: SwerveModuleState):
        if abs(swerve_module_state_.speed) < 0.001:
            self.stop()
            return

        swerve_module_state_ = SwerveModuleState.optimize(swerve_module_state_, self.get_swerve_state().angle)
        self.simAngle = swerve_module_state_.angle.radians()        
        self._angle_pid_controller.setReference(swerve_module_state_.angle.radians(), rev.CANSparkMax.ControlType.kPosition)

        self.speedRPM = swerve_module_state_.speed * 60 / self._chassis._driveMotorConversionFactor
        self.driveFF = 0.0000005
        self._drive_pid_controller.setReference(self.speedRPM, rev.CANSparkMax.ControlType.kVelocity, arbFeedforward=self.driveFF)

    def stop(self):
        self._drive_motor.set(0)
        self._angle_motor.set(0)

    def getSwerveModulePosition(self):
        distance = self.get_drive_position()
        rotation = Rotation2d(self.get_angle_absolute())
        return SwerveModulePosition(distance, rotation)
    
    def simInit(self):
        self.simulation = True
    
    def simUpdate(self, period):
        v = self._drive_motor_encoder.getVelocity() * period
        p = self._drive_motor_encoder.getPosition()
        update = v + p
        update *= self._drive_motor_encoder.getPositionConversionFactor()
        self._drive_motor_encoder.setPosition(update)

        self._angle_motor_encoder.setPosition(self.simAngle)
