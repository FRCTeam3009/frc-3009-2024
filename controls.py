import wpilib

class Controls(object):
    
    def __init__(self, port):
        self.xbox0 = wpilib.XboxController(port)

    def forward(self):
        y = self.xbox0.getLeftY()
        # Negate the value, for some reason it's reversed.
        return y * -1

    def horizontal(self):
        x = self.xbox0.getLeftX()
        return x
    
    def rotate(self):
        rotate = self.xbox0.getRightX()
        # Counter-Clockwise positive, so negate the value.
        return rotate * -1

    def align_zero(self):
        value = self.xbox0.getXButton()
        return value
    
    def launcher(self):
        right = self.xbox0.getRightTriggerAxis()
        left = self.xbox0.getLeftTriggerAxis()

        if left > 0:
            return left
        else:
            return right * -1
        