package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecond;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecondSquared;

import com.ctre.phoenix6.BaseStatusSignal;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.*;
import frc.robot.Robot;

public class SimModule implements ModuleIO {
  private final DCMotorSim _driveMotor = new DCMotorSim(
    LinearSystemId.createDCMotorSystem(
      // convert meters ff to radians ff
      ModuleConstants.DRIVE_KV.times(Meters.per(Radians).of(
        ModuleConstants.DRIVE_WHEEL_CIRCUMFERENCE.in(Meters) / (2 * Math.PI)
      )).magnitude(),

      ModuleConstants.DRIVE_KA.times(Meters.per(Radians).of(
        ModuleConstants.DRIVE_WHEEL_CIRCUMFERENCE.in(Meters) / (2 * Math.PI) 
      )).magnitude()
    ),
    DCMotor.getFalcon500(1),
    ModuleConstants.DRIVE_GEARING
  );

  private final DCMotorSim _turnMotor = new DCMotorSim(
    LinearSystemId.createDCMotorSystem(
      ModuleConstants.TURN_KV.in(VoltsPerRadianPerSecond),
      ModuleConstants.TURN_KA.in(VoltsPerRadianPerSecondSquared)
    ),
    DCMotor.getFalcon500(1),
    ModuleConstants.TURN_GEARING
  );

  private final SimpleMotorFeedforward _driveFF = new SimpleMotorFeedforward(
    0,
    ModuleConstants.DRIVE_KV.magnitude(),
    ModuleConstants.DRIVE_KA.magnitude()
  );

  private final PIDController _drivePID = new PIDController(ModuleConstants.DRIVE_KP.in(VoltsPerMeterPerSecond), 0, 0);
  private final PIDController _turnPID = new PIDController(ModuleConstants.TURN_KP.in(Volts.per(Degree)), 0, 0);

  private double _oldVelocity = 0;

  private String _name = "";

  public SimModule() {
    _turnPID.enableContinuousInput(-180, 180);
  }

  @Override
  public void setName(String moduleName) {
    _name = moduleName;
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

    SmartDashboard.putNumber(_name + " Drive Motor Voltage", volts);
    SmartDashboard.putNumber(_name + " Drive Motor Position", getPosition());
    SmartDashboard.putNumber(_name + " Drive Motor Velocity", getVelocity());
  }

  // only used for testing sysid in sim
  @Override
  public void setTurnVoltage(double volts) {
    _turnMotor.setInputVoltage(volts);
    _turnMotor.update(Robot.kDefaultPeriod);

    SmartDashboard.putNumber(_name + " Turn Motor Voltage", volts);
    SmartDashboard.putNumber(_name + " Turn Motor Position", getAngle().getDegrees());
    SmartDashboard.putNumber(_name + " Turn Motor Velocity", Math.toDegrees(_turnMotor.getAngularVelocityRadPerSec()));
  }
}
