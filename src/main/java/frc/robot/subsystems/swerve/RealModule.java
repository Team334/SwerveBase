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
import frc.lib.CTREUtil;
import frc.lib.FaultLogger;
import frc.lib.Alert.AlertType;
import frc.robot.Robot;
import frc.robot.Constants.SwerveConstants;

public class RealModule implements ModuleIO {
  private final TalonFX _driveMotor;
  private final TalonFX _turnMotor;
  private final CANcoder _turnEncoder;

  private final StatusSignal<Double> _driveVelocity;
  private final StatusSignal<Double> _turnAngle;
  private final StatusSignal<Double> _drivePosition;

  private String _name;

  private boolean _driveMotorConfigError;
  private boolean _turnMotorConfigError;

  private double _oldVelocity = 0;

  public RealModule(int driveMotorId, int turnMotorId, int encoderId) {
    _driveMotor = new TalonFX(driveMotorId);
    _turnMotor = new TalonFX(turnMotorId);
    _turnEncoder = new CANcoder(encoderId);

    _driveVelocity = _driveMotor.getVelocity();
    _driveVelocity.setUpdateFrequency(SwerveConstants.ODOM_FREQUENCY);

    _turnAngle = _turnEncoder.getAbsolutePosition();
    _turnAngle.setUpdateFrequency(SwerveConstants.ODOM_FREQUENCY);

    _drivePosition = _driveMotor.getPosition();
    _drivePosition.setUpdateFrequency(SwerveConstants.ODOM_FREQUENCY);

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
  public double getDriveVelocity() {
    // refreshed by odom thread
    return _driveVelocity.getValue();
  }

  @Override
  public void setAngle(Rotation2d angle) {
    PositionVoltage control = new PositionVoltage(angle.getDegrees());
    _turnMotor.setControl(control);
  }

  @Override
  public Rotation2d getAngle() {
    // refreshed by odom thread
    return Rotation2d.fromDegrees(_turnAngle.getValue());
  }

  @Override
  public double getDrivePosition() {
    // refreshed by odom thread
    return _drivePosition.getValue();
  }

  /**
   * Returns the CTRE StatusSignals to be refreshed periodically in the odom thread. 
   * 
   * <pre>
   * array[0] - Drive Velocity
   * array[1] - Turn Angle
   * array[2] - Drive Position
   * </pre>
   */
  public BaseStatusSignal[] getOdomSignals() {
    return new BaseStatusSignal[] {
      _driveVelocity,
      _turnAngle,
      _drivePosition
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
