package frc.robot.subsystems.swerve.modules;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static frc.lib.subsystem.SelfChecked.sequentialUntil;

import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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

  private final VelocityVoltage _driveVelocitySetter = new VelocityVoltage(0);
  private final PositionVoltage _turnPositionSetter = new PositionVoltage(0);

  private final VoltageOut _driveVoltageSetter = new VoltageOut(0);
  private final VoltageOut _turnVoltageSetter = new VoltageOut(0);

  private String _name;

  private boolean _driveMotorConfigError;
  private boolean _turnMotorConfigError;
  private boolean _turnEncoderConfigError;

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

    // configure all devices
    configureDriveMotor();
    configureTurnMotor();
    configureTurnEncoder();

    _driveVelocitySetter.withSlot(0);
    _turnPositionSetter.withSlot(0);

    FaultLogger.register(_driveMotor);
    FaultLogger.register(_turnMotor);
    FaultLogger.register(_turnEncoder);
  }

  // device configurations
  private void configureDriveMotor() {
    var config = new TalonFXConfiguration();

    _driveMotorConfigError = CTREUtil.configure(_driveMotor, config);
  }

  private void configureTurnMotor() {
    var config = new TalonFXConfiguration();

    _turnMotorConfigError = CTREUtil.configure(_turnMotor, config);
  }

  private void configureTurnEncoder() {
    var config = new CANcoderConfiguration();

    _turnEncoderConfigError = CTREUtil.configure(_turnEncoder, config);
  }


  @Override
  public void setName(String moduleName) {
    _name = moduleName;
  }

  @Override
  public void setVelocity(double velocity, boolean isOpenLoop) {
    _driveVelocitySetter.withVelocity(velocity).withSlot(0);

    if (!isOpenLoop) {
      _driveVelocitySetter.withAcceleration((velocity - _oldVelocity) / Robot.kDefaultPeriod).withSlot(1);
    }

    _driveMotor.setControl(_driveVelocitySetter);

    _oldVelocity = velocity;
  }

  @Override
  public double getVelocity() {
    // refreshed by odom thread
    return _driveVelocity.getValue();
  }

  @Override
  public void setAngle(Rotation2d angle) {
    _turnMotor.setControl(_turnPositionSetter.withPosition(angle.getDegrees()));
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
    _driveMotor.setControl(_driveVoltageSetter.withOutput(volts));
  }

  @Override
  public void setTurnVoltage(double volts) {
    _turnMotor.setControl(_turnVoltageSetter.withOutput(volts));
  }

  @Override
  public Command selfCheck(BiConsumer<String, FaultType> faultAdder, BooleanSupplier hasError) {
    return sequentialUntil(
      hasError,
      runOnce(() -> {
        // check all crucial drive motor faults
        if (_driveMotorConfigError) faultAdder.accept(_name + ": Drive motor failed config, check total faults.", FaultType.ERROR);
        if (_driveMotor.getFault_Hardware().getValue()) faultAdder.accept(_name + ": Drive motor hardware fault, switch device.", FaultType.ERROR);
        if (_driveMotor.getFault_BootDuringEnable().getValue()) faultAdder.accept(_name + ": Drive motor boot during enable, check robot wiring.", FaultType.WARNING);
      }),

      runOnce(() -> {
        // check all crucial turn motor faults
        if (_turnMotorConfigError) faultAdder.accept(_name + ": Turn motor failed config, check total faults.", FaultType.ERROR);
        if (_turnMotor.getFault_Hardware().getValue()) faultAdder.accept(_name + ": Turn motor hardware fault, switch device.", FaultType.ERROR);
        if (_turnMotor.getFault_BootDuringEnable().getValue()) faultAdder.accept(_name + ": Turn motor boot during enable, check robot wiring.", FaultType.WARNING);
      }),

      runOnce(() -> {
        // check all crucial turn encoder faults
        if (_turnEncoderConfigError) faultAdder.accept(_name + ": Turn encoder failed config, check total faults.", FaultType.ERROR);
        if (_turnEncoder.getFault_Hardware().getValue()) faultAdder.accept(_name + ": Turn encoder hardware fault, switch device.", FaultType.ERROR);
        if (_turnEncoder.getFault_BootDuringEnable().getValue()) faultAdder.accept(_name + ": Turn encoder boot during enable, check robot wiring.", FaultType.WARNING);
        if (_turnEncoder.getFault_BadMagnet().getValue()) faultAdder.accept(_name + ": Turn encoder bad magnet.", FaultType.ERROR);
      })
    );
  }
}
