import rev
from wpimath.controller import SimpleMotorFeedforwardMeters
import SparkMotor

class Shooter:
    speakerscale = 0.62
    ampscale = 0.5

    def __init__(self, topId, bottomId, middleId, noteSensorBottom, noteSensorTop, intakeScoop):
        self.kMaxRpm = 5600.0

        self.topMotor = rev.CANSparkMax(topId, rev._rev.CANSparkLowLevel.MotorType.kBrushless)
        self.bottomMotor = rev.CANSparkMax(bottomId, rev._rev.CANSparkLowLevel.MotorType.kBrushless)
        self.middleRamp = rev.CANSparkMax(middleId, rev._rev.CANSparkLowLevel.MotorType.kBrushless) 
        self.topspark = SparkMotor.SparkMotor(self.topMotor)
        self.bottomMotorspark = SparkMotor.SparkMotor(self.bottomMotor)
        self.middleRampSpark = SparkMotor.SparkMotor(self.middleRamp)
        self.noteSensorBottom = noteSensorBottom
        self.noteSensorTop = noteSensorTop
        self.intakeScoopSpark = SparkMotor.SparkMotor(intakeScoop)
        self.wasLookingForNote = False
        self.needsreset = False
    
    def fire(self, value):
        if False:# not self.noteSensorBottom.get() or not self.noteSensorTop.get():
            # We're looking for notes to intake
            # TODO explicitly set launcher motors to not run
            self.intakeScoopSpark._Motor_Pid_.setReference(self.kMaxRpm, rev.CANSparkMax.ControlType.kVelocity)
            self.middleRampSpark._Motor_Pid_.setReference(self.kMaxRpm, rev.CANSparkMax.ControlType.kVelocity)
            self.wasLookingForNote = True
            return
        elif self.wasLookingForNote is True:
            # We grabbed a note but need to let off before firing
            self.intakeScoopSpark._Motor_Pid_.setReference(0, rev.CANSparkMax.ControlType.kVelocity)
            self.middleRampSpark._Motor_Pid_.setReference(0, rev.CANSparkMax.ControlType.kVelocity)
            self.wasLookingForNote = False
            self.needsreset = True

        # At this point we have a note and are preparing to fire
        if self.needsreset:
            self.stop_motors()
            self.intakeScoopSpark._Motor_Pid_.setReference(self.kMaxRpm * -0.1, rev.CANSparkMax.ControlType.kVelocity)
            return
        
        rpm = value * self.kMaxRpm
        topFF = self.topspark.FeedForward.calculate(rpm)
        self.topspark._Motor_Pid_.setReference(rpm * -1, rev.CANSparkMax.ControlType.kVelocity, arbFeedforward=0)
        bottomFF = self.bottomMotorspark.FeedForward.calculate(rpm * -1)
        self.bottomMotorspark._Motor_Pid_.setReference(rpm, rev.CANSparkMax.ControlType.kVelocity, arbFeedforward=0)

        toprpm = self.topspark.encoder.getVelocity()
        bottomrpm = self.bottomMotorspark.encoder.getVelocity()
        if bottomrpm >= rpm and toprpm <= -1 * rpm:
            self.middleRampSpark._Motor_Pid_.setReference(self.kMaxRpm, rev.CANSparkMax.ControlType.kVelocity)
        else: 
            self.middleRampSpark._Motor_Pid_.setReference(0, rev.CANSparkMax.ControlType.kVelocity)

    def stop_motors(self):
        self.bottomMotorspark._Motor_Pid_.setReference(0, rev.CANSparkMax.ControlType.kVelocity)
        self.topspark._Motor_Pid_.setReference(0, rev.CANSparkMax.ControlType.kVelocity)
        self.middleRampSpark._Motor_Pid_.setReference(0, rev.CANSparkMax.ControlType.kVelocity)
        self.intakeScoopSpark._Motor_Pid_.setReference(0, rev.CANSparkMax.ControlType.kVelocity)

    def stop(self):
        self.stop_motors()
        self.needsreset=False

           
