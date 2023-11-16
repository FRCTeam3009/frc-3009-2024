import rev
import phoenix6
import math

class Motor(object):

    def __init__(self, driveID, steeringID, encoderID):
        self.drivemotor = rev.CANSparkMax(driveID, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.steermotor = rev.CANSparkMax(steeringID, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.encoder = phoenix6.CANcoder(encoderID)
        #self.encoder_config = phoenix6.CANcoderConfigurator()
        #self.encoder_config.sensorCoefficient = 2 * math.pi / 4096.0
        #self.encoder_config.unitString = "rad"
        #self.encoder_config.sensorTimeBase = ctre.sensors.SensorTimeBase.PerSecond
        #self.encoder.configAllSettings(self.encoder_config)


    def drive(self, value):
        self.drivemotor.set(value)

    def steer(self, value):
        self.steermotor.set(value)

    def setzero(self):
        self.drivemotor.set(0)
        self.steermotor.set(0)
