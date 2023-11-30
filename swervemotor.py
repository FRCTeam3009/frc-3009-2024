import rev
import ctre
import ctre.sensors
import wpilib

class Motor(object):

    def __init__(self, driveID, steeringID, encoderID):
        self.drivemotor = rev.CANSparkMax(driveID, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.steermotor = rev.CANSparkMax(steeringID, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.encoder = ctre.sensors.CANCoder(encoderID)
        self.encoder_config = ctre.sensors.CANCoderConfiguration()
        self.encoder_config.sensorTimeBase = ctre.sensors.SensorTimeBase.PerSecond
        self.encoder.configAllSettings(self.encoder_config)
        self.timer = wpilib.Timer()
        self.timer.start()


    def drive(self, value):
        self.drivemotor.set(value)

    def steer(self, value):
        self.steermotor.set(value*0.2)

    def setzero(self):
        self.drivemotor.set(0)
        self.steermotor.set(0)

    def align_zero(self):
        position = self.encoder.getAbsolutePosition()
        self.align(position, 0)

    def align(self, position, target):
        offset = 1
        speed = 0

        #position = position - 180 # Convert from 0 to 360 over to -180 to 180

        if position > 180:
            position = -180 + (position % 180)


        difference = target - position

        if self.timer.get() > 0.5:
            print("difference: " + str(difference))
            print("position: " + str(position))
            print(" ")
            self.timer.reset()

        if abs(difference) > offset:
            speed = -0.1

            if difference < 0:
                speed=speed*-1

        self.steer(speed)
