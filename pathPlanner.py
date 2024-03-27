import pathplannerlib
import shooter
import wpilib
import drive_train
import wpimath.geometry
import types
import ntcore

DefaultAutonomousMode = "WCSLeftBlue"

autoModes = [
    "WCSLeftBlue",
    "WCSMiddleBlue",
    "WCSRightBlue",
    "WCSAmpBlue",
    "WCSRightRed",
    "WCSMiddleRed",
    "WCSLeftRed",
    "LeftBlue",
    "MiddleBlue",
    "RightBlue",
    "RightRed",
    "MiddleRed",
    "LeftRed",
    "AmpRed",
    "2NoteAuto"
]

def autonomousDropdown(smartdashboard: ntcore.NetworkTable):
        smartdashboard.putString("autonomousmode/value", DefaultAutonomousMode)
        smartdashboard.putStringArray("autonomousmode/items", autoModes)
    
def selectAuto(smartdashboard):
        autoMode = smartdashboard.getString("autonomousmode/value", DefaultAutonomousMode)
        if autoMode not in autoModes:
            autoMode = DefaultAutonomousMode
        return autoMode

class shootCommand(pathplannerlib.auto.Command):
    def __init__(self, shooter: shooter.Shooter, function: types.FunctionType):
        self.shooter = shooter
        self.function = function
        self.timer = wpilib.Timer()
        self.sensorCount = 0

    def execute(self):
        speed = self.function()
        self.shooter.fire(speed, False, False)

    def end(self, interrupted):
        self.timer.stop()
        self.timer.reset()
        self.shooter.stop()

    def isFinished(self):
        if not self.shooter.hasNote():
            self.timer.start()
        if self.timer.hasElapsed(0.5):
            return True
        return False
    
class lineAprilCommand(pathplannerlib.auto.Command):
    def __init__(self, driveTrain: drive_train.DriveTrain, lineUpToTarget: types.FunctionType, tags):
        self.driveTrain = driveTrain
        self.lineUpToTarget = lineUpToTarget
        self.tags = tags
        self.isDone = False
        self.pose = wpimath.geometry.Pose2d(0, 0, 0)

    def execute(self):
        self.pose = self.lineUpToTarget(self.tags)
        self.driveTrain.Drive(self.pose, False)

    def end(self, interrupted):
        pose = wpimath.geometry.Pose2d(0, 0, 0)
        self.driveTrain.Drive(pose, False)

    def isFinished(self):
        if self.pose.X() != 0 and abs(self.pose.X()) < 0.1:
            return True
    
class lineNoteCommand(pathplannerlib.auto.Command):
    def __init__(self, driveTrain: drive_train.DriveTrain, lineUpToTarget: types.FunctionType, shooter: shooter.Shooter):
        self.driveTrain = driveTrain
        self.lineUpToTarget = lineUpToTarget
        self.isDone = False
        self.shooter = shooter

    def execute(self):
        self.pose = self.lineUpToTarget()
        self.driveTrain.Drive(self.pose, False)
        self.shooter.fire(0, False, False)

    def end(self, interrupted):
        pose = wpimath.geometry.Pose2d(0, 0, 0)
        self.shooter.stop()
        self.driveTrain.Drive(pose, False)

    def isFinished(self):
        if self.shooter.hasNote():
            return True
        else:
            return False
