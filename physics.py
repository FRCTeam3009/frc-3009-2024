import pyfrc.physics.core
import wpilib
import wpilib.simulation
import wpimath.units
import wpimath.system.plant
import phoenix6
import phoenix6.unmanaged
import robot

def create_sim_kraken(conversion):
    return wpilib.simulation.DCMotorSim(wpimath.system.plant.DCMotor.krakenX60FOC(1), 1, conversion)

def update_talon_motor(motor_sim: wpilib.simulation.DCMotorSim, talon_sim: phoenix6.sim.TalonFXSimState, tm_diff):
        talon_sim.set_supply_voltage(wpilib.RobotController.getBatteryVoltage())
        motor_sim.setInputVoltage(talon_sim.motor_voltage)
        motor_sim.update(tm_diff)
        talon_sim.set_raw_rotor_position(wpimath.units.radiansToRotations(motor_sim.getAngularPosition()))
        talon_sim.set_rotor_velocity(wpimath.units.radiansToRotations(motor_sim.getAngularVelocity()))

class PhysicsEngine(pyfrc.physics.core.PhysicsEngine):
    def __init__(self, physics_controller: pyfrc.physics.core.PhysicsInterface, robot: robot.MyRobot):
        self.physics_controller = physics_controller
        self.robot = robot
        driveTrain = self.robot.driveTrain
        self.gyroSim = wpilib.simulation.ADIS16470_IMUSim(driveTrain.gyro)
        driveConversion = driveTrain.fl._drive_module.conversion

        self.flDrive = create_sim_kraken(driveConversion)
        self.flDriveTalon = driveTrain.fl._drive_module.getMotor().sim_state

    def update_sim(self, now: float, tm_diff: float):
        if wpilib.DriverStation.isEnabled():
             phoenix6.unmanaged.feed_enable(100)
             
        self.gyroSim.setGyroAngleY(self.robot.driveTrain.gyro.getAngle(wpilib.ADIS16470_IMU.IMUAxis.kPitch))

        update_talon_motor(self.flDrive, self.flDriveTalon, tm_diff)
        
