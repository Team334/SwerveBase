package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
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
      // convert meters ff to radians ff
      ModuleConstants.DRIVE_KV.times(Meters.per(Radians).of(
        ModuleConstants.DRIVE_WHEEL_CIRCUMFERENCE.in(Meters) / (2 * Math.PI * ModuleConstants.DRIVE_GEARING)
      )).magnitude(),

      ModuleConstants.DRIVE_KA.times(Meters.per(Radians).of(
        ModuleConstants.DRIVE_WHEEL_CIRCUMFERENCE.in(Meters) / (2 * Math.PI * ModuleConstants.DRIVE_GEARING) 
      )).magnitude()
    ),
    DCMotor.getFalcon500(1),
    ModuleConstants.DRIVE_GEARING
  );

  private final DCMotorSim _turnMotor = new DCMotorSim(
    LinearSystemId.createDCMotorSystem(
      ModuleConstants.TURN_KV.in(VoltsPerRadianPerSecond) * ModuleConstants.TURN_GEARING,
      ModuleConstants.TURN_KA.in(VoltsPerRadianPerSecondSquared) * ModuleConstants.TURN_GEARING
    ),
    DCMotor.getFalcon500(1),
    ModuleConstants.TURN_GEARING
  );

  private final SimpleMotorFeedforward _driveFF = new SimpleMotorFeedforward(
    0,
    0,
    0
  );

  private final PIDController _drivePID = new PIDController(0, 0, 0);
  private final PIDController _turnPID = new PIDController(0, 0, 0);

  // only used when testing sysid out in sim
  private DoubleLogEntry _driveMotorVoltage;
  private DoubleLogEntry _driveMotorVelocity;
  private DoubleLogEntry _driveMotorPosition;

  private DoubleLogEntry _turnMotorVoltage;
  private DoubleLogEntry _turnMotorVelocity;
  private DoubleLogEntry _turnMotorPosition;

  private double _oldVelocity = 0;

  public SimModule() {
    _turnPID.enableContinuousInput(-180, 180);

    System.out.println(
      ModuleConstants.DRIVE_KA.times(Meters.per(Radians).of(
        ModuleConstants.DRIVE_WHEEL_CIRCUMFERENCE.in(Meters) / (2 * Math.PI * ModuleConstants.DRIVE_GEARING) 
      )).magnitude()
    );

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
      outVolts = _driveFF.calculate(velocity);
    } else {  
      outVolts = _driveFF.calculate(velocity, (velocity - _oldVelocity) / Robot.kDefaultPeriod);
      outVolts += _drivePID.calculate(getVelocity(), velocity);
    }

    _driveMotor.setInputVoltage(outVolts);
    _driveMotor.update(Robot.kDefaultPeriod);

    _oldVelocity = velocity;
  }

  @Override
  public double getVelocity() {
    return _driveMotor.getAngularVelocityRPM() / 60 * ModuleConstants.DRIVE_WHEEL_CIRCUMFERENCE.in(Meters);
  }


  @Override
  public void setAngle(Rotation2d angle) {
    double outVolts = _turnPID.calculate(getAngle().getDegrees(), angle.getDegrees());

    _turnMotor.setInputVoltage(outVolts);
    _turnMotor.update(Robot.kDefaultPeriod);
  }

  @Override
  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(_turnMotor.getAngularPositionRad());
  }

  @Override
  public double getPosition() {
    return _driveMotor.getAngularPositionRotations() * ModuleConstants.DRIVE_WHEEL_CIRCUMFERENCE.in(Meters);
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
    _turnMotorVelocity.append(Math.toDegrees(_turnMotor.getAngularVelocityRadPerSec()));
    _turnMotorPosition.append(getAngle().getDegrees());
  }
}
