from motorParams import Motorparams
from encoderParams import EncoderParams

class SwerveDriveParams(object):
    def __init__(self, driveMotorParams: Motorparams, angleMotorParams: Motorparams, encoderParams: EncoderParams):
        self._angle_encoder = encoderParams
        self._drive_motor = driveMotorParams
        self._angle_motor = angleMotorParams
