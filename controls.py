import wpilib

class Controls(object):
    
    def __init__(self, port):
        self.xbox0 = wpilib.XboxController(port)

    def forward(self):
        y = self.xbox0.getLeftY()
        return y

    def horizontal(self):
        x = self.xbox0.getLeftX()
        return x
    
    def rotate(self):
        rotate = self.xbox0.getRightX()
        return rotate

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
        