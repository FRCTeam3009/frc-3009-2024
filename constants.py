import wpimath.units

# Front Left Swerve
FLDrive = 43
FLAngle = 20
FLEncoder = 32
FLEncoderOffset = 0.190918

# Rear Left Swerve
RLDrive = 40
RLAngle = 12
RLEncoder = 33
RLEncoderOffset = 0.390869

# Front Right Swerve
FRDrive = 42
FRAngle = 17
FREncoder = 31
FREncoderOffset = -0.127197

# Rear Right Swerve
RRDrive = 41
RRAngle = 15
RREncoder = 30
RREncoderOffset = -0.007324

# Light sensors to detect notes
NoteSensorBottom = 0
NoteSensorTop = 1
NoteSensorFront = 2

autoSwitch = 4

# Shooting system
Scoop = 21
ShooterTop = 22
ShooterBottom = 16
Middle = 18

Climber = 35

LEDs = 2

Servo = 0
ServoClosed = 0.23
ServoOpen = 0.0

buddyServo = 3
buddyServoClosed = 1.0
buddyServoOpen = 0.0

pitchServo = 1
# servo angles
ampServoSetting = 0.01
speakerServoSetting = 0.99
# servo pot limits
speakerLimit = 10.0
ampLimit = 45.0

MaxSpeed = 4.0 # meters per second

speakerHeight = 82.9 # in inches

servoConversion = 1
shooterAngleMin = 46 # Forward, 90 - 44
shooterAngleMax = 68 # Upward, 90 - 22
servoPotMax = 85 # Upward
servoPotMin = 0 # Forward
servoFullRange = 270

#-.83, 0.6 -> 0.83, -0.6 ->  1.8/2 = 0.9  0.4/2 = 0.2
servoMaxValue = 0.9 # Forward
servoMinValue = 0.2 # Upward

ampAngle = 68
trapAngle = 64
speakerAngle = shooterAngleMin

autoDefaultDistance = 2.4 # meters

maxDistanceSpeakerShot = 5.2 # meters
minDistanceSpeakerShot = 2.18 # meters

shooterPitch = 68
shooterSpeed = 0.65

subwooferDistance = wpimath.units.inchesToMeters(34.17) # Literal distance the subwoofer sticks out
subwooferDistanceOffset = wpimath.units.inchesToMeters(34.17 + 11.0) # Closest we can actually reach