package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.*;
import frc.robot.Robot;

public class SimModule implements ModuleIO {
  private final DCMotorSim _driveMotor = new DCMotorSim(
    LinearSystemId.createDCMotorSystem(SwerveModule.DRIVE_KV, SwerveModule.DRIVE_KA),
    DCMotor.getFalcon500(1),
    SwerveModule.DRIVE_GEARING
  );

  private final DCMotorSim _turnMotor = new DCMotorSim(
    LinearSystemId.createDCMotorSystem(SwerveModule.TURN_KV, SwerveModule.TURN_KA),
    DCMotor.getFalcon500(1),
    SwerveModule.TURN_GEARING
  );

  private final SimpleMotorFeedforward _driveFF = new SimpleMotorFeedforward(SwerveModule.DRIVE_KS, SwerveModule.DRIVE_KV, SwerveModule.DRIVE_KA);
  private final PIDController _drivePID = new PIDController(SwerveModule.DRIVE_KP, 0, 0);

  private final PIDController _turnPID = new PIDController(SwerveModule.TURN_KP, 0, 0);

  public SimModule() {
    _turnPID.enableContinuousInput(-180, 180);
  }

  @Override
  public void setDriveTargets(double velocity, double acceleration) {
    double outVolts = _driveFF.calculate(velocity, acceleration) + _drivePID.calculate(getDriveVelocity(), velocity);
    
    _driveMotor.setInputVoltage(outVolts);
    _driveMotor.update(Robot.kDefaultPeriod);
  }

  @Override
  public void setAngle(double angle) {
    double outVolts = _turnPID.calculate(getAngle().getDegrees(), angle);

    _turnMotor.setInputVoltage(outVolts);
    _turnMotor.update(Robot.kDefaultPeriod);
  }

  @Override
  public double getDrivePosition() {
    return (_driveMotor.getAngularPositionRotations() / SwerveModule.DRIVE_GEARING) * SwerveModule.DRIVE_WHEEL_CIRCUMFERENCE.magnitude();
  }

  @Override
  public double getDriveVelocity() {
    return (_driveMotor.getAngularVelocityRPM() / 60 / SwerveModule.DRIVE_GEARING) * SwerveModule.DRIVE_WHEEL_CIRCUMFERENCE.magnitude();
  }

  @Override
  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(_turnMotor.getAngularPositionRotations() / SwerveModule.TURN_GEARING);
  }
}
