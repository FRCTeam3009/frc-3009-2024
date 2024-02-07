import rev
from wpimath.controller import SimpleMotorFeedforwardMeters
import SparkMotor



class Shooter:
    speakerscale = 0.62
    ampscale = 0.5
    def __init__(self, topId, bottomId, middleId):
        self.kMaxRpm = 5600

        self.topMotor = rev.CANSparkMax(topId, rev._rev.CANSparkLowLevel.MotorType.kBrushless)
        self.bottomMotor = rev.CANSparkMax(bottomId, rev._rev.CANSparkLowLevel.MotorType.kBrushless)
        self.middleRamp = rev.CANSparkMax(middleId, rev._rev.CANSparkLowLevel.MotorType.kBrushless) 
        self.topspark = SparkMotor.SparkMotor(self.topMotor)
        self.bottomMotorspark = SparkMotor.SparkMotor(self.bottomMotor)

    
    def fire(self, value):
        rpm = value * self.kMaxRpm
        topFF = self.topspark.FeedForward.calculate(rpm)
        self.topspark._Motor_Pid_.setReference(rpm, rev.CANSparkMax.ControlType.kVelocity, arbFeedforward=topFF)
        bottomFF = self.bottomMotorspark.FeedForward.calculate(rpm * -1)
        self.bottomMotorspark._Motor_Pid_.setReference(rpm * -1, rev.CANSparkMax.ControlType.kVelocity, arbFeedforward=bottomFF)

        toprpm = self.topspark.encoder.getVelocity()
        bottomrpm = self.bottomMotorspark.encoder.getVelocity()
        if toprpm >= rpm and bottomrpm <= -1 * rpm:
            self.middleRamp.set(1.0)
        else: 
            self.middleRamp.set(0)
    
    def stop(self):
        self.bottomMotorspark._Motor_Pid_.setReference(0, rev.CANSparkMax.ControlType.kVelocity)
        self.topspark._Motor_Pid_.setReference(0, rev.CANSparkMax.ControlType.kVelocity)
        self.middleRamp.set(0) # TODO scoop 

           
