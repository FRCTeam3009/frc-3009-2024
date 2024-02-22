import math

class Chassis(object):
    def __init__(self):
        inches_in_meter = 39.37

        # Robot base
        self._wheel_base_width = 24.25 # inches
        self._wheel_base_length = 20.25 # inches
        self._turn_diameter = math.sqrt(self._wheel_base_length**2 + self._wheel_base_width**2) # Pythagorean a^2 + b^2 = c^2
        self._turn_circumference = math.pi * self._turn_diameter / inches_in_meter

        # Swerve Wheels
        self._motor_max_rpm = 5600
        self._k_drive_gear_ratio = 6.75 # L2
        self._k_angle_gear_ratio = 150.0/7.0 # mk4i standard
        self._k_wheel_diameter = 4 / inches_in_meter # 4 inches converted to meters
        self._k_pid_max_error = 1
        self._k_volts = 12
        # https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html
        self._k_s = 0.25 / self._k_volts # voltage to barely move motor
        self._k_v = 2.59 / self._k_volts # https://www.reca.lc/drive
        self._k_a = 0.48 / self._k_volts # https://www.reca.lc/drive

        self._angle_k_v = 8.22 / self._k_volts

        self._wheel_circumference = math.pi * self._k_wheel_diameter
        self._driveMotorConversionFactor = self._wheel_circumference / self._k_drive_gear_ratio # distance per motor rotation (meters)
        self._angleMotorConversionFactor = 2 * math.pi / self._k_angle_gear_ratio # wheel rotation per motor rotation (radians)

        idk = 1.07
        self._turn_meters_per_radian = self._turn_circumference / (idk * math.pi * self._driveMotorConversionFactor)

