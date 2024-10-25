package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecondSquared;

import com.ctre.phoenix6.BaseStatusSignal;

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
    LinearSystemId.createDCMotorSystem(
      // convert meters ff to radians ff
      SwerveModuleConstants.DRIVE_KV.times(Radians.per(Meters).of(
        SwerveModuleConstants.DRIVE_WHEEL_CIRCUMFERENCE.magnitude() / (2 * Math.PI)
      )).magnitude(),

      SwerveModuleConstants.DRIVE_KA.times(Radians.per(Meters).of(
        SwerveModuleConstants.DRIVE_WHEEL_CIRCUMFERENCE.magnitude() / (2 * Math.PI) 
      )).magnitude()
    ),
    DCMotor.getFalcon500(1),
    SwerveModuleConstants.DRIVE_GEARING
  );

  private final DCMotorSim _turnMotor = new DCMotorSim(
    LinearSystemId.createDCMotorSystem(
      SwerveModuleConstants.TURN_KV.in(VoltsPerRadianPerSecond),
      SwerveModuleConstants.TURN_KA.in(VoltsPerRadianPerSecondSquared)
    ),
    DCMotor.getFalcon500(1),
    SwerveModuleConstants.TURN_GEARING
  );

  private final SimpleMotorFeedforward _driveFF = new SimpleMotorFeedforward(
    0,
    SwerveModuleConstants.DRIVE_KV.magnitude(),
    SwerveModuleConstants.DRIVE_KA.magnitude()
  );

  private final PIDController _drivePID = new PIDController(SwerveModuleConstants.DRIVE_KP.magnitude(), 0, 0);
  private final PIDController _turnPID = new PIDController(SwerveModuleConstants.TURN_KP.magnitude(), 0, 0);

  private double _oldVelocity = 0;

  public SimModule() {
    _turnPID.enableContinuousInput(-180, 180);
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
    return _driveMotor.getAngularVelocityRPM() / 60 * SwerveModuleConstants.DRIVE_WHEEL_CIRCUMFERENCE.magnitude();
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
    return _driveMotor.getAngularPositionRotations() * SwerveModuleConstants.DRIVE_WHEEL_CIRCUMFERENCE.magnitude();
  }

  @Override
  public BaseStatusSignal[] getOdomSignals() {
    return new BaseStatusSignal[]{};
  }

  @Override
  public void setDriveVoltage(double volts) {}

  @Override
  public void setTurnVoltage(double volts) {}
}
