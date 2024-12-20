package frc.robot.subsystems.swerve.modules;

import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecondSquared;
import static frc.robot.Constants.SIM_SYSID_LOG_PREFIX;

import com.ctre.phoenix6.BaseStatusSignal;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.*;
import frc.robot.Robot;

public class SimModule implements ModuleIO {
  private String _name = "";

  private final String _sysIdLogPrefix = SIM_SYSID_LOG_PREFIX + "Swerve/";
  
  private final DCMotorSim _driveMotor = new DCMotorSim(
    LinearSystemId.createDCMotorSystem(
      ModuleConstants.DRIVE_KV.in(VoltsPerRadianPerSecond),
      ModuleConstants.DRIVE_KA.in(VoltsPerRadianPerSecondSquared)
    ),
    DCMotor.getFalcon500(1),
    ModuleConstants.DRIVE_GEARING
    // 0.025
  );

  private final DCMotorSim _turnMotor = new DCMotorSim(
    LinearSystemId.createDCMotorSystem(
      ModuleConstants.TURN_KV.in(VoltsPerRadianPerSecond),
      ModuleConstants.TURN_KA.in(VoltsPerRadianPerSecondSquared)
    ),
    DCMotor.getFalcon500(1),
    ModuleConstants.TURN_GEARING
    // 0.004
  );

  private final SimpleMotorFeedforward _driveFF = new SimpleMotorFeedforward(
    0,
    ModuleConstants.DRIVE_KV.in(VoltsPerRadianPerSecond),
    ModuleConstants.DRIVE_KA.in(VoltsPerRadianPerSecondSquared)
  );

  private final PIDController _drivePID = new PIDController(ModuleConstants.DRIVE_KP.in(VoltsPerRadianPerSecond), 0, 0);
  private final PIDController _turnPID = new PIDController(ModuleConstants.TURN_KP.in(Volts.per(Radian)), 0, 0);

  // only used when testing sysid out in sim
  private DoubleLogEntry _driveMotorVoltage;
  private DoubleLogEntry _driveMotorVelocity;
  private DoubleLogEntry _driveMotorPosition;

  private DoubleLogEntry _turnMotorVoltage;
  private DoubleLogEntry _turnMotorVelocity;
  private DoubleLogEntry _turnMotorPosition;

  private double _oldVelocity = 0;

  public SimModule() {
    _turnPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void setName(String moduleName) {
    _name = moduleName;

    DataLog log = DataLogManager.getLog();

    _driveMotorVoltage = new DoubleLogEntry(log, _sysIdLogPrefix + _name + " Drive Motor Voltage");
    _driveMotorVelocity = new DoubleLogEntry(log, _sysIdLogPrefix + _name + " Drive Motor Velocity");
    _driveMotorPosition = new DoubleLogEntry(log, _sysIdLogPrefix + _name + " Drive Motor Position");

    _turnMotorVoltage = new DoubleLogEntry(log, _sysIdLogPrefix + _name + " Turn Motor Voltage");
    _turnMotorVelocity = new DoubleLogEntry(log, _sysIdLogPrefix + _name + " Turn Motor Velocity");
    _turnMotorPosition = new DoubleLogEntry(log, _sysIdLogPrefix + _name + " Turn Motor Position");
  }

  @Override
  public void setVelocity(double velocity, boolean isOpenLoop) {
    double outVolts;

    if (isOpenLoop) {
      // NOTE!!!: In sim, the falcon 500 input current -> output torque conversion is small so more voltage is necessary across
      // the windings and that's why the acceleration feedforward is used.
      // Also, acceleration cannot be calculated as just (vel-oldVel) / .02s, because acceleration isn't constant in the timestep 
      // (it decreases as velocity increases), so this method of the simple motor feedforward will calculate the correct acceleration
      // setpoint to reach vel in dt (this is similar to chassis speeds discretize)
      outVolts = _driveFF.calculate(_oldVelocity, velocity, Robot.kDefaultPeriod);

      // outVolts = _driveFF.calculate(velocity); // poor tracking
    } else {  
      outVolts = _driveFF.calculate(_oldVelocity, velocity, Robot.kDefaultPeriod);
      outVolts += _drivePID.calculate(getVelocity(), velocity);
    }

    _driveMotor.setInputVoltage(outVolts);
    _driveMotor.update(Robot.kDefaultPeriod);

    _oldVelocity = velocity;
  }

  @Override
  public double getVelocity() {
    return _driveMotor.getAngularVelocityRadPerSec();
  }


  @Override
  public void setAngle(Rotation2d angle) {
    double outVolts = _turnPID.calculate(getAngle().getRadians(), angle.getRadians());

    _turnMotor.setInputVoltage(outVolts);
    _turnMotor.update(Robot.kDefaultPeriod);
  }

  @Override
  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(_turnMotor.getAngularPositionRad());
  }

  @Override
  public double getPosition() {
    return _driveMotor.getAngularPositionRad();
  }

  @Override
  public BaseStatusSignal[] getOdomSignals() {
    return new BaseStatusSignal[]{};
  }

  // only used for testing sysid in sim
  @Override
  public void setDriveVoltage(double volts) {
    _driveMotor.setInputVoltage(volts);
    _driveMotor.update(Robot.kDefaultPeriod);

    _driveMotorVoltage.append(volts);
    _driveMotorVelocity.append(getVelocity());
    _driveMotorPosition.append(getPosition());
  }

  // only used for testing sysid in sim
  @Override
  public void setTurnVoltage(double volts) {
    _turnMotor.setInputVoltage(volts);
    _turnMotor.update(Robot.kDefaultPeriod);

    _turnMotorVoltage.append(volts);
    _turnMotorVelocity.append(_turnMotor.getAngularVelocityRadPerSec());
    _turnMotorPosition.append(getAngle().getRadians());
  }
}
