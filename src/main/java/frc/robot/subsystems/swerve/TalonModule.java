package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.lib.FaultLogger;

public class TalonModule implements ModuleIO {
  private final TalonFX _driveMotor;
  private final TalonFX _turnMotor;

  private final StatusSignal<Double> _driveVelocityGetter;
  private final StatusSignal<Double> _turnPositionGetter;

  private final VelocityVoltage _driveVelocitySetter = new VelocityVoltage(0);
  private final PositionVoltage _turnPositionSetter = new PositionVoltage(0);

  public TalonModule(String name, int driveMotorId, int turnMotorId) {
    _driveMotor = new TalonFX(driveMotorId);
    _turnMotor = new TalonFX(turnMotorId);

    _driveVelocityGetter = _driveMotor.getVelocity();
    _turnPositionGetter = _turnMotor.getPosition();

    // TODO: Add all motor configs here

    FaultLogger.register(name + " Drive Motor", _driveMotor);
    FaultLogger.register(name + " Turn Motor", _turnMotor);
  }

  @Override
  public void setDriveTargets(double velocity, double acceleration) {
    _driveVelocitySetter.withVelocity(velocity).withAcceleration(acceleration);
    _driveMotor.setControl(_driveVelocitySetter);
  }

  @Override
  public void setTurnAngle(double angle) {
    _turnPositionSetter.withPosition(angle);
    _turnMotor.setControl(_turnPositionSetter);
  }

  @Override
  public double getDriveVelocity() {
    return _driveVelocityGetter.refresh().getValue();
  }

  @Override
  public double getTurnPosition() {
    return _turnPositionGetter.refresh().getValue();
  }
}
