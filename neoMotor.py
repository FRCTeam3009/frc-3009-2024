import rev

class neoMotor:
    def __init__(self, id, isInverted, conversionFactor):
        self.simulation = False
        self.pidController = None
        self.conversionFactor = conversionFactor
        self.canMotor = rev.CANSparkMax(id, rev.CANSparkLowLevel.MotorType.kBrushless)
        self.canMotor.setInverted(isInverted)
        self.motorEncoder = self.canMotor.getEncoder()
        self.motorEncoder.setPositionConversionFactor(self.conversionFactor)
        self.motorEncoder.setVelocityConversionFactor(self.conversionFactor / 60)

        self.simPosition = 0.0

    def getMotor(self):
        return self.canMotor
    
    def setupPid(self, p, i, d, iZone, FF, pidWrapping, outputLow, outputHigh, minInput, maxInput):
        self.pidController = self.canMotor.getPIDController()
        self.pidController.setP(p)
        self.pidController.setI(i)
        self.pidController.setD(d)
        self.pidController.setIZone(iZone)
        self.pidController.setFF(FF)
        if pidWrapping:
            self.pidController.setPositionPIDWrappingEnabled(pidWrapping)
        self.pidController.setOutputRange(outputLow, outputHigh)
        if minInput and maxInput:
            self.pidController.setPositionPIDWrappingMinInput(minInput)
            self.pidController.setPositionPIDWrappingMaxInput(maxInput)

    def getPosition(self):
        if self.simulation:
            return self.simPosition * self.conversionFactor
        
        return self.motorEncoder.getPosition()
    
    def getVelocity(self):
        if self.simulation:
            return 0
        
        return self.motorEncoder.getVelocity()
    
    def setPosition(self, position):
        self.simPosition = position
        self.motorEncoder.setPosition(position)

    def setReference(self, rads, controlType, arbFF = 0):
        self.simPosition = rads / self.conversionFactor
        self.pidController.setReference(rads, controlType, arbFeedforward=arbFF)

    def stop(self):
        self.canMotor.set(0)

    def getPositionConversionFactor(self):
        return self.conversionFactor

    def setInverted(self, value):
        self.canMotor.setInverted(value)

    def getP(self):
        return self.pidController.getP()
    
    def getI(self):
        return self.pidController.getI()
    
    def getD(self):
        return self.pidController.getD()
    
    def set(self, value):
        self.canMotor.set(value)

    def simInit(self):
        self.simulation = True

    def simUpdate(self, period):
        # this is a position PID controller, so it just goes to the position, no updates.
        return