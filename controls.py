import wpilib

class Controls(object):
    
    def __init__(self, port0, port1):
        self.driverController = wpilib.XboxController(port0)
        self.shooterController = wpilib.XboxController(port1)

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
        value=self.shooterController.getStartButton()
        return value
    
    def reset_gyro(self):
        value=self.driverController.getXButton()
        return value
    
    def reset_goal(self):
        return self.driverController.getBackButton()
    
    def climber(self):
        return self.shooterController.getLeftX()
    
    def target_amp(self):
        return self.driverController.getPOV() == 90
    
    def target_subwoofer(self):
        return self.driverController.getPOV() == 270
    
    def target_stage(self):
        return self.driverController.getPOV() == 0
    
    def target_closest(self):
        return self.driverController.getPOV() == 180

    def turbo(self):
        return self.driverController.getRightBumper()
    
    def slow(self):
        return self.driverController.getLeftBumper()
    
    def push_trap(self):
        return self.driverController.getYButtonPressed()
    
    def override(self):
        return self.shooterController.getYButton()

    def reverseOverride(self):
        return self.shooterController.getXButton()
