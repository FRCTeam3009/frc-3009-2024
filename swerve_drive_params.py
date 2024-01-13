from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.geometry import Translation2d
import wpilib

class SwerveDriveParams(object):
    def __init__(self, name_):
        self._angle_encoder = {'id': None, 'offset' : None}
        self._drive_motor = {'id' : None, 'pid_p' : None, 'pid_i' : None, 'pid_d' : None}
        self._angle_motor = {'id' : None, 'pid_p' : None, 'pid_i' : None, 'pid_d' : None}
        self._name = name_

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


    def setup_angle_encoder(self, id_, offset_):
        self._angle_encoder['id'] = id_
        self._angle_encoder['offset'] = offset_

    def setup_drive_motor(self, id_, pid_p_ = 0, pid_i_ = 0, pid_d_ = 0):
        self._drive_motor['id'] = id_
        self._drive_motor['pid_p'] = pid_p_
        self._drive_motor['pid_i'] = pid_i_
        self._drive_motor['pid_d'] = pid_d_

    def setup_angle_motor(self, id_, pid_p_ = 0, pid_i_ = 0, pid_d_ = 0):
        self._angle_motor['id'] = id_
        self._angle_motor['pid_p'] = pid_p_
        self._angle_motor['pid_i'] = pid_i_
        self._angle_motor['pid_d'] = pid_d_

    def get_angle_encoder(self, param_ = None):
        if param_:
            return self._angle_encoder[param_]
        return self._angle_encoder
    
    def get_drive_motor(self, param_ = None):
        if param_:
            return self._drive_motor[param_]
        return self._drive_motor
    
    def get_angle_motor(self, param_ = None):
        if param_:
            return self._angle_motor[param_]
        return self._angle_motor

class SwerveDrives(object):
    def __init__(self):
        self._fl = SwerveDriveParams("fl")
        self._fr = SwerveDriveParams("fr")
        self._rl = SwerveDriveParams("rl")
        self._rr = SwerveDriveParams("rr")

class Chassis(object):
    def __init__(self):
        self._wheel_base_width = 21
        self._wheel_base_length = 25

class robot_description(object):
    def __init__(self):
        self._swerve_drives = SwerveDrives()
        self._chassis = Chassis()
        self._drive_kinematics = SwerveDrive4Kinematics(
            Translation2d(self._chassis._wheel_base_width / 2.0, self._chassis._wheel_base_length / 2.0),
            Translation2d( self._chassis._wheel_base_width/ 2.0, - self._chassis._wheel_base_length/ 2.0),
            Translation2d(-self._chassis._wheel_base_width / 2.0, self._chassis._wheel_base_length / 2.0),
            Translation2d(- self._chassis._wheel_base_width/ 2.0, -self._chassis._wheel_base_length / 2.0),
        )
        # TODO update to Pidgeon
        #self._gyro = wpilib.ADXRS450_Gyro()
