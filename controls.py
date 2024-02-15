import wpilib

class Controls(object):
    
    def __init__(self, port0, port1):
        self.xbox0 = wpilib.XboxController(port0)
        self.xbox1 = wpilib.XboxController(port1)

    def forward(self):
        y = self.xbox0.getLeftY()
        # Negate the value, for some reason it's reversed.
        return y * -1

    def horizontal(self):
        x = self.xbox0.getLeftX()
        # Negate because CC positive?
        return x * -1
    
    def rotate(self):
        rotate = self.xbox0.getRightX()
        # Counter-Clockwise is positive
        rotate *= -1
        return rotate
    
    def shootspeaker(self):
        right = self.xbox1.getRightTriggerAxis()
        if right > 0.5:
            return 1.0
        else: 
            return 0.0
    
    def shootamp(self):
        left = self.xbox1.getLeftTriggerAxis()
        if left > 0.5:
            return 1.0
        else: 
            return 0.0
   
    def rotate_to_target(self):
        value=self.xbox0.getStartButton()
        return value
    
    def reset_gyro(self):
        value=self.xbox0.getXButton()
        return value
    
    def scoop_speed(self):
        value = self.xbox1.getLeftY()
        return value * -1
    
    def middle_speed(self):
        value = self.xbox1.getRightY()
        return value * -1
    
    def reset_goal(self):
        return self.xbox0.getBackButton()
    
    def climber(self):
        return self.xbox1.getLeftX()
    
    def target_amp(self):
        return self.xbox0.getPOV() == 90
    
    def target_subwoofer(self):
        return self.xbox0.getPOV() == 270
    
    def target_stage(self):
        return self.xbox0.getPOV() == 0
    
    def target_closest(self):
        return self.xbox0.getPOV() == 180

    def turbo(self):
        return self.xbox0.getRightBumper()
    
    def slow(self):
        return self.xbox0.getLeftBumper()
    
    def push_trap(self):
        return self.xbox0.getYButtonPressed()

