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
        # Limit the output
        rotate *= 0.1
        return rotate
    
    def launcher(self):
        right = self.xbox1.getRightTriggerAxis()
        left = self.xbox1.getLeftTriggerAxis()

        if left > 0:
            return left * -1
        else:
            return right
        
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