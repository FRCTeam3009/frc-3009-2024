import rev
import SparkMotor
import constants
import wpimath.units
import wpilib

class Shooter:
    speakerspeed_close = 0.65 #0.8 #0.62
    speakerspeed_far = 0.8
    ampspeed = 0.30
    trapspeed = 0.45

    def __init__(self, topId, bottomId, middleId, noteSensorBottom, noteSensorTop, intakeScoop):
        self.kMaxRpm = 5600.0

        self.topMotor = rev.CANSparkMax(topId, rev._rev.CANSparkLowLevel.MotorType.kBrushless)
        self.bottomMotor = rev.CANSparkMax(bottomId, rev._rev.CANSparkLowLevel.MotorType.kBrushless)
        self.middleRamp = rev.CANSparkMax(middleId, rev._rev.CANSparkLowLevel.MotorType.kBrushless)
        self.topspark = SparkMotor.SparkMotor(self.topMotor)
        self.bottomMotorspark = SparkMotor.SparkMotor(self.bottomMotor)
        self.topspark._Motor_Pid_.setP(6e-5)
        self.topspark._Motor_Pid_.setI(5e-7)
        self.topspark._Motor_Pid_.setD(0)
        self.bottomMotorspark._Motor_Pid_.setP(6e-5)
        self.bottomMotorspark._Motor_Pid_.setI(5e-7)
        self.bottomMotorspark._Motor_Pid_.setD(0)
        self.middleRampSpark = SparkMotor.SparkMotor(self.middleRamp)
        self.noteSensorBottom = noteSensorBottom
        self.noteSensorTop = noteSensorTop
        self.intakeScoopSpark = SparkMotor.SparkMotor(intakeScoop)
        self.wasLookingForNote = False
        self.needsreset = False
        self.isLaunching = False
        self.scoopScale = 0.4

    def hasNote(self):
         return (self.noteSensorBottom.get() or self.noteSensorTop.get())
    
    def fire(self, value, override, reverseOverride):

        self.rpmTimer = wpilib.Timer()

        seen = self.hasNote()
        if reverseOverride:
            self.intakeScoopSpark._Motor_Pid_.setReference(-self.kMaxRpm, rev.CANSparkMax.ControlType.kVelocity)
            self.middleRampSpark._Motor_Pid_.setReference(-self.kMaxRpm/2, rev.CANSparkMax.ControlType.kVelocity)
            self.bottomMotorspark._Motor_Pid_.setReference(-self.kMaxRpm/6, rev.CANSparkMax.ControlType.kVelocity, arbFeedforward=0.02)
            self.topspark._Motor_Pid_.setReference(self.kMaxRpm/6, rev.CANSparkMax.ControlType.kVelocity, arbFeedforward=0.02)
            return

        if override:
            seen = not seen
        if not seen and not self.isLaunching:
            # We're looking for notes to intake
            self.intakeOn()
            self.wasLookingForNote = True
            return
        elif self.wasLookingForNote is True:
            # We grabbed a note but need to let off before firing
            self.intakeScoopSpark._Motor_Pid_.setReference(0, rev.CANSparkMax.ControlType.kVelocity)
            self.middleRampSpark._Motor_Pid_.setReference(0, rev.CANSparkMax.ControlType.kVelocity)
            self.wasLookingForNote = False
            if not override:
                self.needsreset = True
            else:
                self.stop_motors()

        # At this point we have a note and are preparing to fire
        if self.needsreset:
            self.stop_motors()
            self.intakeScoopSpark._Motor_Pid_.setReference(self.kMaxRpm * -0.1, rev.CANSparkMax.ControlType.kVelocity)
            return
        
        self.isLaunching = True
        rpm = value * self.kMaxRpm
        self.topspark._Motor_Pid_.setReference(rpm * -1, rev.CANSparkMax.ControlType.kVelocity, arbFeedforward=0)
        self.bottomMotorspark._Motor_Pid_.setReference(rpm, rev.CANSparkMax.ControlType.kVelocity, arbFeedforward=0)

        toprpm = self.topspark.encoder.getVelocity()
        bottomrpm = self.bottomMotorspark.encoder.getVelocity()
        rpmBufferUp = rpm + 50
        rpmBufferDown = rpm - 50
        if bottomrpm >= rpmBufferDown and bottomrpm < rpmBufferUp and toprpm <= -1 * rpmBufferDown and toprpm > -1 * rpmBufferUp:
            self.middleRampSpark._Motor_Pid_.setReference(self.kMaxRpm, rev.CANSparkMax.ControlType.kVelocity)
        else: 
            self.middleRampSpark._Motor_Pid_.setReference(0, rev.CANSparkMax.ControlType.kVelocity)

    def stop_motors(self):
        '''self.bottomMotorspark._Motor_Pid_.setReference(0, rev.CANSparkMax.ControlType.kVelocity)
        self.topspark._Motor_Pid_.setReference(0, rev.CANSparkMax.ControlType.kVelocity)'''
        self.topMotor.set(0)
        self.bottomMotor.set(0)
        self.middleRampSpark._Motor_Pid_.setReference(0, rev.CANSparkMax.ControlType.kVelocity)
        self.intakeScoopSpark._Motor_Pid_.setReference(0, rev.CANSparkMax.ControlType.kVelocity)

    def stop(self):
        self.stop_motors()
        self.needsreset=False
        self.isLaunching = False

    def intakeOn(self):       
        self.intakeScoopSpark._Motor_Pid_.setReference(self.kMaxRpm, rev.CANSparkMax.ControlType.kVelocity)
        self.middleRampSpark._Motor_Pid_.setReference(self.kMaxRpm * self.scoopScale, rev.CANSparkMax.ControlType.kVelocity)
        self.bottomMotorspark._Motor_Pid_.setReference(0, rev.CANSparkMax.ControlType.kVelocity)
        self.topspark._Motor_Pid_.setReference(0, rev.CANSparkMax.ControlType.kVelocity)

class speedAngle:
    def __init__(self, speed, angle):
        self.speed = speed
        self.angle = angle

angleSpeed = {
    constants.subwooferDistanceOffset: speedAngle(0.65, 64), # Shortest distance
    2.15: speedAngle(61, .78),
    2.41: speedAngle(57, 0.78),
    2.60: speedAngle(55, 0.78),
    2.92: speedAngle(52, 0.78),
    3.25: speedAngle(48.5, 0.78),
    3.58: speedAngle(46, 0.768),
    #3.92: speedAngle(46, 0.72),
    wpimath.units.inchesToMeters(180): speedAngle(0.8, 46),
}

def lookUpAngleSpeed(distance):
    distancesList = angleSpeed.keys()
    closestKey = constants.subwooferDistanceOffset
    closestAbs = 100000000000
    for variable in distancesList:
        absDistance = abs(variable - distance)
        if absDistance < closestAbs:
            closestKey = variable
            closestAbs = absDistance

    return angleSpeed[closestKey]

