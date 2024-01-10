import wpilib

class Joystick(object):
    
    def __init__(self, port):
        self.joystick = wpilib.XboxController(port)

    def forward(self):
        y = self.joystick.getLeftY()
        return y

    def horizontal(self):
        x = self.joystick.getLeftX()
        return x
    
    def rotate(self):
        rotate = self.joystick.getRightX()
        return rotate

    def align_zero(self):
        value = self.joystick.getXButton()
        return value
    
    def launcher(self):
        right = self.joystick.getRightTriggerAxis()
        left = self.joystick.getLeftTriggerAxis()

        if left > 0:
            return left
        else:
            return right * -1
        