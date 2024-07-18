package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.FaultLogger;

public class TalonModule implements ModuleIO {
  private final TalonFX _driveMotor;
  private final TalonFX _turnMotor;
  private final CANcoder _turnEncoder;

  private final StatusSignal<Double> _driveVelocityGetter;
  private final StatusSignal<Double> _encoderPositionGetter;

  private final VelocityVoltage _driveVelocitySetter = new VelocityVoltage(0);
  private final PositionVoltage _turnPositionSetter = new PositionVoltage(0);

  public TalonModule(String name, int driveMotorId, int turnMotorId, int encoderId) {
    _driveMotor = new TalonFX(driveMotorId);
    _turnMotor = new TalonFX(turnMotorId);
    _turnEncoder = new CANcoder(encoderId);

    _driveVelocityGetter = _driveMotor.getVelocity();
    _encoderPositionGetter = _turnEncoder.getAbsolutePosition();

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
  public Rotation2d getTurnPosition() {
    return Rotation2d.fromRotations(_encoderPositionGetter.refresh().getValueAsDouble());
  }
}
