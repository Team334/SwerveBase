package frc.robot.subsystems.swerve.modules;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static frc.lib.subsystem.SelfChecked.sequentialUntil;

import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.CTREUtil;
import frc.lib.FaultLogger;
import frc.lib.FaultsTable.FaultType;
import frc.robot.Constants.CAN;
import frc.robot.Constants.ModuleConstants;

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

  private final double _driveMotorResistance = DCMotor.getFalcon500(1).rOhms; 

  private String _name;

  private boolean _driveMotorConfigError = false;
  private boolean _turnMotorConfigError = false;
  private boolean _turnEncoderConfigError = false;

  public RealModule(int driveMotorId, int turnMotorId, int encoderId) {
    _driveMotor = new TalonFX(driveMotorId);
    _turnMotor = new TalonFX(turnMotorId);
    _turnEncoder = new CANcoder(encoderId);

    _driveVelocity = _driveMotor.getVelocity();
    _turnAngle = _turnEncoder.getAbsolutePosition();
    _drivePosition = _driveMotor.getPosition();

    if (CAN.REDUCE_CONTROL_LATENCY) {
      // stops the control requests from being sent periodically, this will reduce control request latency
      // but it's risky because if the time between the last setControl call is >20ms the motors will disable
      _driveVelocitySetter.withUpdateFreqHz(0);
      _turnPositionSetter.withUpdateFreqHz(0);
      _driveVoltageSetter.withUpdateFreqHz(0);
      _turnVoltageSetter.withUpdateFreqHz(0);
    }

    // configure all devices
    configureDriveMotor();
    configureTurnMotor();
    configureTurnEncoder();

    _driveMotorConfigError |= CTREUtil.attempt(() -> _driveMotor.optimizeBusUtilization(), _driveMotor);
    _turnMotorConfigError |= CTREUtil.attempt(() -> _turnMotor.optimizeBusUtilization(), _turnMotor);
    _turnEncoderConfigError |= CTREUtil.attempt(() -> _turnEncoder.optimizeBusUtilization(), _turnEncoder);

    FaultLogger.register(_driveMotor);
    FaultLogger.register(_turnMotor);
    FaultLogger.register(_turnEncoder);
  }

  // device configurations
  private void configureDriveMotor() {
    var config = new TalonFXConfiguration();

    // open-loop slot (0)
    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = ModuleConstants.DRIVE_KS.in(Volts);
    slot0Configs.kV = ModuleConstants.DRIVE_KV.in(Volts.per(RevolutionsPerSecond));

    // closed-loop slot (1)
    var slot1Configs = new Slot1Configs();
    slot1Configs.kS = ModuleConstants.DRIVE_KS.in(Volts);
    slot1Configs.kV = ModuleConstants.DRIVE_KV.in(Volts.per(RevolutionsPerSecond));
    slot1Configs.kP = ModuleConstants.DRIVE_KP.in(Volts.per(RevolutionsPerSecond));

    var feedback = new FeedbackConfigs();
    feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    feedback.SensorToMechanismRatio = ModuleConstants.DRIVE_GEARING;

    var motorOutput = new MotorOutputConfigs();
    motorOutput.NeutralMode = NeutralModeValue.Coast;

    // TODO
    var currentLimits = new CurrentLimitsConfigs();
    currentLimits.StatorCurrentLimit = ModuleConstants.DRIVE_STATOR_CURRENT_LIMIT.in(Amps);
    currentLimits.SupplyCurrentLimit = ModuleConstants.SUPPLY_CURRENT_LIMIT.in(Amps);
    currentLimits.StatorCurrentLimitEnable = true;
    currentLimits.SupplyCurrentLimitEnable = true;

    config.withSlot0(slot0Configs)
          .withSlot1(slot1Configs)
          .withFeedback(feedback)
          .withMotorOutput(motorOutput)
          .withCurrentLimits(currentLimits);

    _driveMotorConfigError |= CTREUtil.attempt(() -> _driveMotor.getConfigurator().apply(config), _driveMotor);
  }

  private void configureTurnMotor() {
    var config = new TalonFXConfiguration();

    // closed-loop slot (0)
    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = ModuleConstants.TURN_KP.in(Volts.per(Rotations));

    var feedback = new FeedbackConfigs();
    feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    feedback.FeedbackRemoteSensorID = _turnEncoder.getDeviceID();

    var closedLoopGeneral = new ClosedLoopGeneralConfigs();
    closedLoopGeneral.ContinuousWrap = true;

    var motorOutput = new MotorOutputConfigs();
    motorOutput.NeutralMode = NeutralModeValue.Brake;

    // TODO
    var currentLimits = new CurrentLimitsConfigs();
    currentLimits.SupplyCurrentLimit = 0;
    currentLimits.SupplyCurrentLimitEnable = true;

    config.withSlot0(slot0Configs)
          .withFeedback(feedback)
          .withMotorOutput(motorOutput)
          .withCurrentLimits(currentLimits)
          .withClosedLoopGeneral(closedLoopGeneral);

    _turnMotorConfigError |= CTREUtil.attempt(() -> _turnMotor.getConfigurator().apply(config), _turnMotor);
  }

  private void configureTurnEncoder() {
    var config = new CANcoderConfiguration();

    var magnetSensor = new MagnetSensorConfigs(); 
    
    // refresh with enc offset that was set in tuner x
    _turnEncoderConfigError |= CTREUtil.attempt(() -> _turnEncoder.getConfigurator().refresh(magnetSensor), _turnEncoder);
    
    magnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    magnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    config.withMagnetSensor(magnetSensor);

    _turnEncoderConfigError |= CTREUtil.attempt(() -> _turnEncoder.getConfigurator().apply(config), _turnEncoder);
  }


  @Override
  public void setName(String moduleName) {
    _name = moduleName;
  }

  @Override
  public void setVelocity(double velocity, boolean isOpenLoop) {
    // convert to rotations/second (this won't be needed in 2025)
    double rps = Units.radiansToRotations(velocity);

    if (isOpenLoop) {
      _driveVelocitySetter.withVelocity(rps).withSlot(0);
    } else {
      // gonna use pathplanner 2025 torque-current
      _driveVelocitySetter.withVelocity(rps).withFeedForward(0 * _driveMotorResistance).withSlot(1);
    }

    _driveMotor.setControl(_driveVelocitySetter);
  }

  @Override
  public double getVelocity() {
    // refreshed by odom thread
    return Units.rotationsToRadians(_driveVelocity.getValue());
  }

  @Override
  public void setAngle(Rotation2d angle) {
    _turnMotor.setControl(_turnPositionSetter.withPosition(angle.getRotations()));
  }

  @Override
  public Rotation2d getAngle() {
    // refreshed by odom thread
    return Rotation2d.fromRotations(_turnAngle.getValue());
  }

  @Override
  public double getPosition() {
    // refreshed by odom thread
    return Units.rotationsToRadians(_drivePosition.getValue());
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
