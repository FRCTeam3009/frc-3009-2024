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
    def __init__(self, shooter: shooter.Shooter, scale):
        self.shooter = shooter
        self.scale = scale
        self.timer = wpilib.Timer()
        self.sensorCount = 0
    def execute(self):
        self.shooter.fire(self.scale, False, False)

    def end(self, interrupted):
        #self.shooter.stop()
        pass
    def isFinished(self):
        self.timer.start()
        if self.timer.hasElapsed(5.0):
            self.timer.stop()
            self.timer.reset()
            self.shooter.stop()
            #self.shooter.fire(self.scale, False, False)
            return True
        elif self.shooter.hasNote() is False:
            self.sensorCount += 1
            if self.timer.hasElapsed(1.0):
                self.shooter.stop()
                self.shooter.fire(self.scale, False, False)
        if self.sensorCount > 10:
            if self.timer.hasElapsed(4.0):
                self.timer.stop()
                self.timer.reset()
                self.shooter.stop()
                self.shooter.fire(self.scale, False, False)
                return True
        return False
    
class lineAprilCommand(pathplannerlib.auto.Command):
    def __init__(self, driveTrain: drive_train.DriveTrain, lineUpToTarget: types.FunctionType, tags):
        self.driveTrain = driveTrain
        self.lineUpToTarget = lineUpToTarget
        self.tags = tags
        self.isDone = False
    def execute(self):
        self.pose = self.lineUpToTarget(self.tags)
        self.driveTrain.Drive(self.pose, False)
    def end(self, interrupted):
        #pose = wpimath.geometry.Pose2d(0, 0, 0)
        return
    def isFinished(self):
        if abs(self.pose.x()) < 0.1:
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
        if self.shooter.hasNote(False):
            return True
        else:
            return False
