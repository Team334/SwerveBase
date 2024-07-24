package frc.robot.subsystems.swerve;

import java.util.function.BiConsumer;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.FaultLogger;
import frc.lib.Alert.AlertType;

public class TalonModule implements ModuleIO {
  private final TalonFX _driveMotor;
  private final TalonFX _turnMotor;
  private final CANcoder _turnEncoder;

  private final StatusSignal<Double> _drivePositionGetter;
  private final StatusSignal<Double> _driveVelocityGetter;
  private final StatusSignal<Double> _encoderPositionGetter;

  private final VelocityVoltage _driveVelocitySetter = new VelocityVoltage(0);
  private final PositionVoltage _turnPositionSetter = new PositionVoltage(0);

  private final int _driveMotorId;
  private final int _turnMotorId;

  private final String _name;

  public TalonModule(String name, int driveMotorId, int turnMotorId, int encoderId) {
    _name = name;

    _driveMotor = new TalonFX(driveMotorId);
    _turnMotor = new TalonFX(turnMotorId);
    _turnEncoder = new CANcoder(encoderId);

    _drivePositionGetter = _driveMotor.getPosition();
    _driveVelocityGetter = _driveMotor.getVelocity();
    _encoderPositionGetter = _turnEncoder.getAbsolutePosition();

    // TODO: Add all motor configs here
    _driveMotorId = FaultLogger.register(_name + "/Drive Motor", _driveMotor);
    _turnMotorId = FaultLogger.register(_name + "/Turn Motor", _turnMotor);
  }

  @Override
  public void setDriveTargets(double velocity, double acceleration) {
    _driveVelocitySetter.withVelocity(velocity).withAcceleration(acceleration);
    _driveMotor.setControl(_driveVelocitySetter);
  }

  @Override
  public void setAngle(double angle) {
    _turnPositionSetter.withPosition(angle);
    _turnMotor.setControl(_turnPositionSetter);
  }

  @Override
  public double getDrivePosition() {
    return _drivePositionGetter.refresh().getValue();
  }

  @Override
  public double getDriveVelocity() {
    return _driveVelocityGetter.refresh().getValue();
  }

  @Override
  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(_encoderPositionGetter.refresh().getValueAsDouble());
  }

  @Override
  public Command selfCheck(BiConsumer<String, AlertType> alerter) {
    return Commands.runOnce(() -> {
      var driveMotorFaults = FaultLogger.getFaults(_driveMotorId);
      driveMotorFaults.forEach(f -> alerter.accept(f.toString(), f.alertType()));

      if (driveMotorFaults.size() == 0) {
        alerter.accept(_name + ": Drive Motor Good", AlertType.INFO);
      }

      var turnMotorFaults = FaultLogger.getFaults(_turnMotorId);
      turnMotorFaults.forEach(f -> alerter.accept(f.toString(), f.alertType()));

      if (turnMotorFaults.size() == 0) {
        alerter.accept(_name + ": Turn Motor Good", AlertType.INFO);
      }
    });
  }
}
