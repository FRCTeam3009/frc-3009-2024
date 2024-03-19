import pyfrc.physics.core
import pyfrc.physics.drivetrains
import wpilib
import wpilib.simulation
import wpimath.units
import wpimath.system.plant
import phoenix6
import phoenix6.unmanaged
import robot

def create_sim_kraken(conversion):
    return wpilib.simulation.DCMotorSim(wpimath.system.plant.DCMotor.krakenX60FOC(1), 1, conversion)

def update_swerve_module(motor_sim: wpilib.simulation.DCMotorSim, talon_sim: phoenix6.sim.TalonFXSimState, cancoder_sim: phoenix6.sim.CANcoderSimState, tm_diff):
        talon_sim.set_supply_voltage(wpilib.RobotController.getBatteryVoltage())
        motor_sim.setInputVoltage(talon_sim.motor_voltage)
        motor_sim.update(tm_diff)
        talon_sim.set_raw_rotor_position(wpimath.units.radiansToRotations(motor_sim.getAngularPosition()))
        talon_sim.set_rotor_velocity(wpimath.units.radiansToRotations(motor_sim.getAngularVelocity()))

        cancoder_sim.set_supply_voltage(wpilib.RobotController.getBatteryVoltage())
        cancoder_sim.set_raw_position(wpimath.units.radiansToRotations(motor_sim.getAngularPosition()))
        cancoder_sim.set_velocity(wpimath.units.radiansToRotations(motor_sim.getAngularVelocity()))
        

class PhysicsEngine(pyfrc.physics.core.PhysicsEngine):
    def __init__(self, physics_controller: pyfrc.physics.core.PhysicsInterface, robot: robot.MyRobot):
        self.physics_controller = physics_controller
        self.robot = robot
        driveTrain = self.robot.driveTrain
        self.gyroSim = wpilib.simulation.ADIS16470_IMUSim(driveTrain.gyro)
        driveConversion = 1.0 / driveTrain.fl._drive_module.conversion

        self.flDrive = create_sim_kraken(driveConversion)
        self.flDriveTalon = driveTrain.fl._drive_module.getMotor().sim_state
        self.flEncoder = driveTrain.fl._encoder.sim_state

        self.frDrive = create_sim_kraken(driveConversion)
        self.frDriveTalon = driveTrain.fr._drive_module.getMotor().sim_state
        self.frEncoder = driveTrain.fr._encoder.sim_state

        self.rlDrive = create_sim_kraken(driveConversion)
        self.rlDriveTalon = driveTrain.rl._drive_module.getMotor().sim_state
        self.rlEncoder = driveTrain.rl._encoder.sim_state

        self.rrDrive = create_sim_kraken(driveConversion)
        self.rrDriveTalon = driveTrain.rr._drive_module.getMotor().sim_state
        self.rrEncoder = driveTrain.rr._encoder.sim_state

    def update_sim(self, now: float, tm_diff: float):
        if wpilib.DriverStation.isEnabled():
             phoenix6.unmanaged.feed_enable(100)
             
        self.gyroSim.setGyroAngleY(self.robot.driveTrain.gyro.getAngle(wpilib.ADIS16470_IMU.IMUAxis.kPitch))

        update_swerve_module(self.flDrive, self.flDriveTalon, self.flEncoder, tm_diff)
        update_swerve_module(self.frDrive, self.frDriveTalon, self.frEncoder, tm_diff)
        update_swerve_module(self.rlDrive, self.rlDriveTalon, self.rlEncoder, tm_diff)
        update_swerve_module(self.rrDrive, self.rrDriveTalon, self.rrEncoder, tm_diff)

        
