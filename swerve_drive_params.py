from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.geometry import Translation2d
from motorParams import Motorparams
from encoderParams import EncoderParams

class SwerveDriveParams(object):
    def __init__(self, driveMotorParams: Motorparams, angleMotorParams: Motorparams, encoderParams: EncoderParams):
        self._angle_encoder = encoderParams
        self._drive_motor = driveMotorParams
        self._angle_motor = angleMotorParams

        self._k_drive_gear_ratio = 6.75 #L2
        self._k_angle_gear_ratio = 150/7 #mk4i standard
        self._k_wheel_diameter = 4
        self._k_pid_max_error = 1
        self._k_volts = 12
        # TODO: Do we need to divide by volts?
        # https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html
        self._k_s = 0.25 / self._k_volts # TODO: voltage to barely move motor
        self._k_v = 2.59 / self._k_volts # https://www.reca.lc/drive
        self._k_a = 0.48 / self._k_volts # https://www.reca.lc/drive


class Chassis(object):
    def __init__(self):
        self._wheel_base_width = 21
        self._wheel_base_length = 25

class robot_description(object):
    def __init__(self, fl, fr, rl, rr):
        self.fl = fl 
        self.fr = fr
        self.rl = rl
        self.rr = rr

        self._chassis = Chassis()
        self._drive_kinematics = SwerveDrive4Kinematics(
            Translation2d(self._chassis._wheel_base_width / 2.0, self._chassis._wheel_base_length / 2.0),
            Translation2d( self._chassis._wheel_base_width/ 2.0, - self._chassis._wheel_base_length/ 2.0),
            Translation2d(-self._chassis._wheel_base_width / 2.0, self._chassis._wheel_base_length / 2.0),
            Translation2d(- self._chassis._wheel_base_width/ 2.0, -self._chassis._wheel_base_length / 2.0),
        )

