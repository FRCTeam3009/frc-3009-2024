import pathplannerlib
import shooter
import wpilib
import drive_train
import wpimath.geometry




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
    def __init__(self, driveTrain: drive_train.DriveTrain, cameraValues):
        self.driveTrain = driveTrain
        self.cameraValues = cameraValues
    def execute(self):
        return
    def end(self):
        pose = wpimath.geometry.Pose2d(0, 0, 0)
        self.driveTrain.Drive(pose, False)
    def isFinished(self):
        return True
    
# TODO finish
class lineNoteCommand(pathplannerlib.auto.Command):
    def __init__(self, driveTrain: drive_train.DriveTrain, cameraValues):
        self.driveTrain = driveTrain
        self.cameraValues = cameraValues
    def execute(self):
        return
    def end(self):
        pose = wpimath.geometry.Pose2d(0, 0, 0)
        self.driveTrain.Drive(pose, False)
    def isFinished(self):
        return True