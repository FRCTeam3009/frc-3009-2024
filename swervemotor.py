import rev
import phoenix6
import wpilib

class Motor(object):

    def __init__(self, driveID, steeringID, encoderID, encoderOffset, inverted=False):
        self.drivemotor = rev.CANSparkMax(driveID, rev._rev.CANSparkLowLevel.MotorType.kBrushless)
        self.drivemotor.setInverted(inverted)
        self.steermotor = rev.CANSparkMax(steeringID, rev._rev.CANSparkLowLevel.MotorType.kBrushless)
        self.encoder = phoenix6.hardware.CANcoder(encoderID)
        configs = phoenix6.configs.MagnetSensorConfigs()
        configs.magnet_offset(encoderOffset)
        configs.with_absolute_sensor_range(phoenix6.)
        self.encoder.configurator.apply(configs)
        self.encoder_config.initializationStrategy.BootToAbsolutePosition
        self.encoder.configAllSettings(self.encoder_config)
        self.timer = wpilib.Timer()
        self.timer.start()


    def drive(self, value):
        scale = 0.5
        self.drivemotor.set(value*scale)

    def steer(self, value):
        scale = 0.2
        self.steermotor.set(value*scale)

    def setzero(self):
        self.drivemotor.set(0)
        self.steermotor.set(0)

    def align_zero(self):
        position = self.encoder.getAbsolutePosition()
        speed = Motor.align(position, 0)
        self.steer(speed)

    def align(position, target):
        offset = 0.1
        speed = 0

        #position = position - 180 # Convert from 0 to 360 over to -180 to 180

        if position > 180:
            position = -180 + (position % 180)



        difference = target - position

        if abs(difference) > offset:
            speed = 0.3
            bufferzone = 10
            if abs(difference) <= bufferzone:
                speed = speed*(abs(difference)/bufferzone)

            if difference >= 0:
                speed=speed*-1
        else:
            speed = 0.1
            if difference >= 0:
                speed=speed * -1
        

        return speed