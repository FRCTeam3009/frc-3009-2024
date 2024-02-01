from motorParams import Motorparams
from encoderParams import EncoderParams

class SwerveDriveParams(object):
    def __init__(self, driveMotorParams: Motorparams, angleMotorParams: Motorparams, encoderParams: EncoderParams):
        self._angle_encoder = encoderParams
        self._drive_motor = driveMotorParams
        self._angle_motor = angleMotorParams

        self._k_drive_gear_ratio = 6.75 #L2
        self._k_angle_gear_ratio = 150.0/7.0 #mk4i standard
        self._k_wheel_diameter = 4
        self._k_pid_max_error = 1
        self._k_volts = 12
        # TODO: Do we need to divide by volts?
        # https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html
        self._k_s = 0.25 / self._k_volts # TODO: voltage to barely move motor
        self._k_v = 2.59 / self._k_volts # https://www.reca.lc/drive
        self._k_a = 0.48 / self._k_volts # https://www.reca.lc/drive

        self._angle_k_v = 8.22 / self._k_volts
