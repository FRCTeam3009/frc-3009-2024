import phoenix6
import wpimath.units
import constants

class krakenMotor():
    def __init__(self, id, isInverted, conversionFactor):
        self.pidController = None
        self.canMotor = phoenix6.hardware.TalonFX(id)
        self.config = phoenix6.configs.TalonFXConfiguration()
        self.conversion = conversionFactor
        if isInverted:
            self.config.motor_output.inverted = phoenix6.configs.config_groups.InvertedValue.CLOCKWISE_POSITIVE
        else:
            self.config.motor_output.inverted = phoenix6.configs.config_groups.InvertedValue.COUNTER_CLOCKWISE_POSITIVE

        self.config.motor_output.neutral_mode = phoenix6.configs.config_groups.NeutralModeValue.BRAKE
        self.config.feedback.sensor_to_mechanism_ratio = self.conversion
        self.canMotor.configurator.apply(self.config)

        self.simulation = False
        self.simPosition = phoenix6.units.rotation(0)
        self.simRPS = phoenix6.units.rotations_per_second(0)

    def getMotor(self):
        return self.canMotor
    
    def setupPid(self, p, i, d, s, a, v):
        self.config.slot0.k_p = p
        self.config.slot0.k_i = i
        self.config.slot0.k_d = d
        self.config.slot0.k_s = s
        self.config.slot0.k_a = a
        self.config.slot0.k_v = v

        # motion_magic_configs = self.config.motion_magic
        # motion_magic_configs.motion_magic_acceleration = (constants.MaxSpeed / self.conversion) * 2 # order of magnitude above vel
        # motion_magic_configs.motion_magic_jerk = motion_magic_configs.motion_magic_acceleration * 5 # order of magnitude above acc

        self.canMotor.configurator.apply(self.config)

    def getPosition(self):
        if self.simulation:
            return self.simPosition
        
        return self.canMotor.get_position().value        
    
    def getVelocity(self):
        if self.simulation:
            return self.simRPS
        return self.canMotor.get_velocity().value
    
    def setPosition(self, position: phoenix6.units.rotation):
        if self.simulation:
            self.simPosition = position
        else:
            self.canMotor.set_position(position)

    def setReference(self, RPM, arbFF = 0):
        RPS = RPM / 60
        self.simRPS = phoenix6.units.rotations_per_second(RPS)
        request = phoenix6.controls.VelocityVoltage(0).with_slot(0)
        self.canMotor.set_control(request.with_velocity(RPS).with_feed_forward(arbFF))
        #request = phoenix6.controls.MotionMagicVelocityVoltage(0)
        #self.canMotor.set_control(request.with_velocity(RPS))

    def stop(self):
        self.setReference(0, 0)

    def getPositionConversionFactor(self):
        return self.conversion

    def getP(self):
        return self.config.slot0.k_p
    
    def getI(self):
        return self.config.slot0.k_i
    
    def getD(self):
        return self.config.slot0.k_d
    
    def set(self, value):
        self.setReference(value)

    def simInit(self):
        self.simulation = True

    def simUpdate(self, period):
        distance = self.simRPS * self.getPositionConversionFactor() * period
        self.simPosition += distance