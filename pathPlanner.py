import pathplannerlib
import shooter
import wpilib
import drive_train
import wpimath.geometry
import types




class shootCommand(pathplannerlib.auto.Command):
    def __init__(self, shooter: shooter.Shooter, scale):
        self.shooter = shooter
        self.scale = scale
        self.timer = wpilib.Timer()
    def execute(self):
        self.shooter.fire(self.scale, False, False)

    def end(self):
        self.shooter.stop()
    def isFinished(self):
        if self.shooter.hasNote() is False:
            self.timer.start()
            if self.timer.hasElapsed(0.5):
                self.timer.stop()
                self.timer.reset()
                return True
        return False
    
# TODO finish
class lineAprilCommand(pathplannerlib.auto.Command):
    def __init__(self, driveTrain: drive_train.DriveTrain, lineUpToTarget: types.FunctionType, tags):
        self.driveTrain = driveTrain
        self.lineUpToTarget = lineUpToTarget
        self.tags = tags
        self.isDone = False
    def execute(self):
        return
    def end(self):
        #pose = wpimath.geometry.Pose2d(0, 0, 0)
        pose = self.lineUpToTarget(self.tags)
        while abs(pose.x()) >= 0.1:
            self.driveTrain.Drive(pose, False)
            pose = self.lineUpToTarget(self.tags)
        self.isDone = True
    def isFinished(self):
        return self.isDone
    
# TODO finish
class lineNoteCommand(pathplannerlib.auto.Command):
    def __init__(self, driveTrain: drive_train.DriveTrain, lineUpToTarget: types.FunctionType):
        self.driveTrain = driveTrain
        self.lineUpToTarget = lineUpToTarget
        self.isDone = False
    def execute(self):
        return
    def end(self):
        #pose = wpimath.geometry.Pose2d(0, 0, 0)
        pose = self.lineUpToTarget()
        while abs(pose.x()) >= 0.1:
            self.driveTrain.Drive(pose, False)
            pose = self.lineUpToTarget()
        self.isDone = True
    def isFinished(self):
        return self.isDone
