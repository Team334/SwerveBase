package frc.robot.subsystems.swerve;

import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Alert.AlertType;
import frc.lib.subsystem.SelfChecked;

public class SwerveModule implements SelfChecked {
  private final ModuleIO _io;

  private final String _name;

  private SwerveModuleState _desiredState = new SwerveModuleState();

  private ControlMode _controlMode = ControlMode.OPEN_LOOP;

  /** Represents the control over the module's drive motor. */
  public static enum ControlMode {
    /** Drives this module open loop. */
    OPEN_LOOP,

    /** Drives this module closed loop. */
    CLOSED_LOOP
  }

  public SwerveModule(String name, ModuleIO io) {
    _io = io;
    _name = name;
    
    if (_io instanceof RealModule) ((RealModule) _io).setName(_name);
  }

  /** Returns the last desired set state for this module. */
  public SwerveModuleState getDesiredState() {
    return _desiredState;
  }

  /** Get the measured state of this module. */
  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(_io.getDriveVelocity(), _io.getAngle());
  }

  /** Set the target state for this module. */
  public void setModuleState(SwerveModuleState state, ControlMode controlMode) {
    _desiredState = SwerveModuleState.optimize(state, _io.getAngle());
    _controlMode = controlMode;
  }

  /** Updates this module periodically so it can reach the target state. */
  public void periodic() {    
    setDrive(_desiredState.speedMetersPerSecond);
    setAngle(_desiredState.angle);
  }

  /** Set the target velocity and acceleration for this module. */
  public void setDrive(double velocity) {
    _io.setVelocity(velocity, _controlMode == ControlMode.OPEN_LOOP ? true : false);
  }

  /** Set the target angle for this module. */ 
  public void setAngle(Rotation2d angle) {
    _io.setAngle(angle);
  }

  @Override
  public Command selfCheck(BiConsumer<String, AlertType> alerter, BooleanSupplier hasError) {
    return _io.selfCheck(alerter, hasError);
  }
}
