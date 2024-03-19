speaker_speed = 0.78 # previously tried 0.8 and 0.62
amp_speed = 0.30
trap_speed = 0.45

# Max range that the potentiometer reads.
pot_lower_limit = 0.0
pot_upper_limit = 60.0

amp_servo_setting = 0.01
amp_servo_pot = 55.0

speaker_servo_setting = 0.99
speaker_servo_pot = 5.0

def pot_angle_to_servo_setting(pot: float):
    '''
    This function converts pot values to servo values.
    When the servo is set to go to 1.0, the pot reads 0. (pot_lower_limit)
    When the servo is set to go to 0.0, the pot reads 60. (pot_upper_limit)
    '''
    if pot > pot_upper_limit:
        pot = pot_upper_limit

    if pot < pot_lower_limit:
        pot = pot_lower_limit
    
    pot_range = pot_upper_limit - pot_lower_limit
    shifted = pot - pot_range
    normalized = shifted / pot_range
    
    return normalized * -1


class ShooterAngle(object):
    def __init__(self, speed: float, angle: float) -> None:
        self.speed = speed
        self.angle = angle

    def is_shooter_at_angle(self, pot_angle):
        servo_angle = pot_angle_to_servo_setting(pot_angle)
        buffer = 0.15
        upper_limit = self.angle + buffer
        lower_limit = self.angle - buffer
        if servo_angle >= lower_limit and servo_angle <= upper_limit:
            return True
        
        return False
    
def new_amp_angle():
    return ShooterAngle(amp_speed, amp_servo_setting, amp_servo_pot)

def new_trap_angle():
    return ShooterAngle(trap_speed, amp_servo_setting, amp_servo_pot)

def new_speaker_angle():
    return ShooterAngle(speaker_speed, speaker_servo_setting, speaker_servo_pot)