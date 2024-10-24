package frc.robot.subsystems.swerve;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static frc.lib.subsystem.SelfChecked.sequentialUntil;

import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.CTREUtil;
import frc.lib.FaultLogger;
import frc.lib.FaultsTable.FaultType;
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
  public double getVelocity() {
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
  public double getPosition() {
    // refreshed by odom thread
    return _drivePosition.getValue();
  }

  @Override
  public BaseStatusSignal[] getOdomSignals() {
    return new BaseStatusSignal[] {
      _driveVelocity,
      _turnAngle,
      _drivePosition
    };
  }
  
  @Override
  public void setDriveVoltage(double volts) {
    VoltageOut control = new VoltageOut(volts);

    _driveMotor.setControl(control);
  }

  @Override
  public void setTurnVoltage(double volts) {
    VoltageOut control = new VoltageOut(volts);

    _turnMotor.setControl(control);
  }

  @Override
  public Command selfCheck(BiConsumer<String, FaultType> faultAdder, BooleanSupplier hasError) {
    return sequentialUntil(
      hasError,
      runOnce(() -> {
        // check all crucial drive motor faults
        if (_driveMotorConfigError) faultAdder.accept(_name + ": Drive motor failed config, check total faults.", FaultType.ERROR);
        if (_driveMotor.getFault_Hardware().getValue()) faultAdder.accept(_name + ": Drive motor hardware fault, switch device.", FaultType.ERROR);
        if (_driveMotor.getFault_BootDuringEnable().getValue()) faultAdder.accept(_name + ": Drive motor boot during enable, check robot wiring.", FaultType.ERROR);
      }),

      runOnce(() -> {
        // check all crucial turn motor faults
        if (_turnMotorConfigError) faultAdder.accept(_name + ": Turn motor failed config, check total faults.", FaultType.ERROR);
        if (_turnMotor.getFault_Hardware().getValue()) faultAdder.accept(_name + ": Turn motor hardware fault, switch device.", FaultType.ERROR);
        if (_turnMotor.getFault_BootDuringEnable().getValue()) faultAdder.accept(_name + ": Turn motor boot during enable, check robot wiring.", FaultType.ERROR);
      }),

      runOnce(() -> {
        // check all crucial turn encoder faults
        if (_turnEncoder.getFault_Hardware().getValue()) faultAdder.accept(_name + ": Turn encoder hardware fault, switch device.", FaultType.ERROR);
        if (_turnEncoder.getFault_BootDuringEnable().getValue()) faultAdder.accept(_name + ": Turn encoder boot during enable, check robot wiring.", FaultType.ERROR);
        if (_turnEncoder.getFault_BadMagnet().getValue()) faultAdder.accept(_name + ": Turn encoder bad magnet.", FaultType.ERROR);
      })
    );
  }
}
