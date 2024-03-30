import wpilib

class Controls(object):
    
    def __init__(self, port0, port1):
        self.driverController = wpilib.XboxController(port0)
        self.shooterController = wpilib.XboxController(port1)
        self.isRobotRelative = False
        self.AprilMode = True

    def forward(self):
        y = self.driverController.getLeftY()
        # Negate the value, for some reason it's reversed.
        return pow(y, 3) * -1

    def horizontal(self):
        x = self.driverController.getLeftX()
        # Negate because CC positive?
        return pow(x, 3) * -1
    
    def rotate(self):
        rotate = self.driverController.getRightX()
        # Counter-Clockwise is positive
        rotate *= -1
        return pow(rotate, 3)
    
    def shootspeaker(self):
        right = self.shooterController.getRightTriggerAxis()
        if right > 0.5:
            return 1.0
        else: 
            return 0.0
    
    def shootamp(self):
        left = self.shooterController.getLeftTriggerAxis()
        if left > 0.5:
            return 1.0
        else: 
            return 0.0
   
    def note_pickup(self):
        value=self.driverController.getRightBumper()
        return value
    
    def reset_gyro(self):
        value=self.driverController.getBButton()
        return value
    
    def reset_goal(self):
        return self.driverController.getBackButton()
    
    def climber(self):
        return self.shooterController.getLeftY() * -1
    
    def target_amp(self):
        return self.driverController.getPOV() == 90 # RIGHT
    
    def target_speaker(self):
        return self.driverController.getPOV() == 0 # UP
    
    def target_stage(self):
        return self.driverController.getPOV() == 270 # LEFT
    
    def target_closest(self):
        return self.driverController.getPOV() == 180 # DOWN

    def turbo(self):
        if self.driverController.getLeftTriggerAxis() > 0.5:
            return 1.0
        else:
            return 0.0
    
    def slow(self):
        if self.driverController.getLeftBumper():
            return 1.0
        else:
            return 0.0
    
    def push_trap(self):
        return self.driverController.getYButtonPressed()
    
    def override(self):
        return self.shooterController.getYButton()

    def reverseOverride(self):
        return self.shooterController.getXButton()

    def shootTrap(self):
        return self.shooterController.getLeftBumper()
    
    def robotView(self):
        if self.driverController.getAButtonPressed():
            self.isRobotRelative = not self.isRobotRelative
        return self.isRobotRelative
    
    def shooterangle(self):
        value = self.shooterController.getRightY() * -1
        return (value + 1)/2
    
    def target_lock(self):
        if self.driverController.getRightTriggerAxis() > 0.5:
            return True
        else:
            return False
    
    def ampPitch(self):
        return self.shooterController.getAButton()
    
    def speakerPitch(self):
        return self.shooterController.getBButton()
    
    def buddyBar(self):
        return self.driverController.getXButtonPressed()
    
    def trapPitch(self):
        return self.shooterController.getRightBumper()
        