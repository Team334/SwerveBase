package frc.robot.subsystems.swerve;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static frc.lib.subsystem.SelfChecked.sequentialUntil;

import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.FaultLogger;
import frc.lib.Alert.AlertType;
import frc.lib.util.CTREUtil;
import frc.robot.Robot;

public class RealModule implements ModuleIO {
  private final TalonFX _driveMotor;
  private final TalonFX _turnMotor;
  private final CANcoder _turnEncoder;

  private final StatusSignal<Double> _drivePositionGetter;
  private final StatusSignal<Double> _driveVelocityGetter;
  private final StatusSignal<Double> _turnPositionGetter;

  private String _name;

  private boolean _driveMotorConfigError;
  private boolean _turnMotorConfigError;

  private double _oldVelocity = 0;

  public RealModule(int driveMotorId, int turnMotorId, int encoderId) {
    _driveMotor = new TalonFX(driveMotorId);
    _turnMotor = new TalonFX(turnMotorId);
    _turnEncoder = new CANcoder(encoderId);

    _drivePositionGetter = _driveMotor.getPosition();
    _driveVelocityGetter = _driveMotor.getVelocity();
    _turnPositionGetter = _turnEncoder.getAbsolutePosition();

    // TODO: Add all motor configs here
    _driveMotorConfigError = CTREUtil.configure(_driveMotor, null);
    _turnMotorConfigError = CTREUtil.configure(_turnMotor, null);

    FaultLogger.register(_driveMotor);
    FaultLogger.register(_turnMotor);
  }

  /** Gives this io the name of its parent SwerveModule to use when displaying info. */
  public void setName(String name) {
    _name = name;
  }

  @Override
  public void setVelocity(double velocity, boolean isOpenLoop) {
    VelocityVoltage control = new VelocityVoltage(velocity).withAcceleration((velocity - _oldVelocity) / Robot.kDefaultPeriod);
    control.Slot = isOpenLoop ? 0 : 1;
    _driveMotor.setControl(control);

    _oldVelocity = velocity;
  }

  @Override
  public void setAngle(Rotation2d angle) {
    PositionVoltage control = new PositionVoltage(angle.getDegrees());
    _turnMotor.setControl(control);
  }

  @Override
  public double getDrivePosition() {
    return _drivePositionGetter.getValue();
  }

  @Override
  public double getDriveVelocity() {
    return _driveVelocityGetter.getValue();
  }

  @Override
  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(_turnPositionGetter.getValue());
  }

  /**
   * Returns the CTRE StatusSignals relevant to this module as an array. These signals must be refreshed
   * periodically for the io getters of this module to update in value.
   * 
   * <pre>
   * array[0] - Drive Position
   * array[1] - Drive Velocity
   * array[2] - Turn Position
   * </pre>
   */
  public BaseStatusSignal[] getSignals() {
    return new BaseStatusSignal[] {
      _drivePositionGetter,
      _driveVelocityGetter,
      _turnPositionGetter
    };
  }
  
  @Override
  public Command selfCheck(BiConsumer<String, AlertType> alerter, BooleanSupplier hasError) {
    return sequentialUntil(
      hasError,
      runOnce(() -> {
        // check all crucial drive motor faults
        if (_driveMotorConfigError) alerter.accept(_name + ": Drive motor failed config, check total faults.", AlertType.ERROR);
        if (_driveMotor.getFault_Hardware().getValue()) alerter.accept(_name + ": Drive motor hardware fault, switch device.", AlertType.ERROR);
        if (_driveMotor.getFault_BootDuringEnable().getValue()) alerter.accept(_name + ": Drive motor boot during enable, check robot wiring.", AlertType.ERROR);
      }),

      runOnce(() -> {
        // check all crucial turn motor faults
        if (_turnMotorConfigError) alerter.accept(_name + ": Turn motor failed config, check total faults.", AlertType.ERROR);
        if (_turnMotor.getFault_Hardware().getValue()) alerter.accept(_name + ": Turn motor hardware fault, switch device.", AlertType.ERROR);
        if (_turnMotor.getFault_BootDuringEnable().getValue()) alerter.accept(_name + ": Turn motor boot during enable, check robot wiring.", AlertType.ERROR);
      }),

      runOnce(() -> {
        // check all crucial turn encoder faults
        if (_turnEncoder.getFault_Hardware().getValue()) alerter.accept(_name + ": Turn encoder hardware fault, switch device.", AlertType.ERROR);
        if (_turnEncoder.getFault_BootDuringEnable().getValue()) alerter.accept(_name + ": Turn encoder boot during enable, check robot wiring.", AlertType.ERROR);
        if (_turnEncoder.getFault_BadMagnet().getValue()) alerter.accept(_name + ": Turn encoder bad magnet.", AlertType.ERROR);
      }),

      runOnce(() -> {
        alerter.accept(_name + ": IO configuration was successful!", AlertType.INFO);
        alerter.accept(_name + ": Drive motor looks good!", AlertType.INFO);
        alerter.accept(_name + ": Turn motor looks good!", AlertType.INFO);
        alerter.accept(_name + ": Turn encoder looks good!", AlertType.INFO);
      })
    );
  }
}
