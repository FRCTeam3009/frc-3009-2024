import wpilib
import math

class Joystick(object):
    
    def __init__(self, port):
        self.joystick = wpilib.XboxController(port)

    def forward(self):
        x = self.joystick.getLeftX()
        dx = Joystick.deadzone(x)
        return dx

    def horizontal(self):    
        y = self.joystick.getLeftY()
        dy = Joystick.deadzone(y)
        return dy
    
    def rotate(self):
        rotate = self.joystick.getRightX()
        drotate = Joystick.deadzone(rotate)
        return drotate
    
    def deadzone(value):
        ignore = 0.0 # our new controllers are awesome
        OG = value
        if value < 0:
            value *= -1
        if value < ignore:
            return 0
        else:
            return OG
        