import rev
from wpimath.controller import SimpleMotorFeedforwardMeters
class SparkMotor:
    def __init__(self, motor:rev.CANSparkMax):
        
       _k_volts = 12.0
       k_v = 2.59 / _k_volts
       p_value = 6e-5
       i_value = 1e-6
       d_value = 0

       self._Motor_Pid_ = motor.getPIDController()
       self._Motor_Pid_.setP(p_value)
       self._Motor_Pid_.setI(i_value)
       self._Motor_Pid_.setD(d_value)
       self._Motor_Pid_.setIZone(0)
       self._Motor_Pid_.setFF(0)
       self._Motor_Pid_.setOutputRange(-1.0, 1.0)

       self._k_s = 0.25 / _k_volts
       self._k_v = 2.59 / _k_volts
       self._k_a = 0.48 / _k_volts
       self.FeedForward = SimpleMotorFeedforwardMeters(self._k_s, self._k_v, self._k_a)
       self.encoder = motor.getEncoder()