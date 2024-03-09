import phoenix6

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
        self.simPosition = 0

        '''self.motorEncoder = self.canMotor.getEncoder()
        self.motorEncoder.setPositionConversionFactor(conversionFactor)
        self.motorEncoder.setVelocityConversionFactor(self.motorEncoder.getPositionConversionFactor() / 60)'''
    
    def simInit(self):
        self.simulation = True

    def getMotor(self):
        return self.canMotor
    
    def setupPid(self, p, i, d, s, a, v):
        self.config.slot0.k_p = p
        self.config.slot0.k_i = i
        self.config.slot0.k_d = d
        self.config.slot0.k_s = s
        self.config.slot0.k_a = a
        self.config.slot0.k_v = v

        self.canMotor.configurator.apply(self.config)

    def getPosition(self):
        if self.simulation:
            return self.simPosition
        
        return self.canMotor.get_position().value        
    
    def getVelocity(self):
        return self.canMotor.get_velocity().value
    
    def setPosition(self, position):
        if self.simulation:
            self.simPosition = position
        else:
            self.canMotor.set_position(position)

    def setReference(self, RPM, arbFF = 0):
        RPS = RPM / 60
        request = phoenix6.controls.VelocityVoltage(0).with_slot(0)
        self.canMotor.set_control(request.with_velocity(RPS).with_feed_forward(arbFF))

    def stop(self):
        self.setReference(0, 0)

    def getPositionConversionFactor(self):
        return self.conversion

    def setInverted(self, isInverted):
        if isInverted:
            self.config.motor_output.inverted = phoenix6.configs.config_groups.InvertedValue.CLOCKWISE_POSITIVE
        else:
            self.config.motor_output.inverted = phoenix6.configs.config_groups.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.canMotor.configurator.apply(self.config)

    def getP(self):
        return self.config.slot0.k_p
    
    def getI(self):
        return self.config.slot0.k_i
    
    def getD(self):
        return self.config.slot0.k_d
    
    def set(self, value):
        self.setReference(value)