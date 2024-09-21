package frc.robot.subsystems.swerve;

import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Alert.AlertType;
import frc.lib.subsystem.SelfChecked;

public class SwerveModule implements SelfChecked {
  private final ModuleIO _io;

  private final String _name;

  private SwerveModuleState _desiredState = new SwerveModuleState();

  private boolean _isOpenLoop = false; 

  public SwerveModule(String name, ModuleIO io) {
    _io = io;
    _name = name;
    
    if (_io instanceof RealModule) ((RealModule) _io).setName(_name);
  }

  public String getName() {
    return _name;
  }

  public ModuleIO getIO() {
    return _io;
  }

  /** Returns the last desired set state for this module. */
  public SwerveModuleState getDesiredState() {
    return _desiredState;
  }

  /** Get the measured state of this module. */
  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(_io.getVelocity(), _io.getAngle());
  }

  /**
   * Returns the position (distance, angle) of this module. This method
   * should only be called inside the odom thread to avoid thread safety issues,
   * or enclosed in a lock shared by the odom thread.
   */
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(_io.getPosition(), _io.getAngle());
  }

  /** 
   * Set the desired target state for this module.
   * 
   * @param state The target state.
   * @param isOpenLoop Whether the velocity of the state is met in an open loop (FF only) or closed loop manner (FF + PID).
   * @param allowTurnInPlace If this is true, all rotation states will be ignored for velocities of 0.
   */
  public void setModuleState(SwerveModuleState state, boolean isOpenLoop, boolean allowTurnInPlace) {
    _desiredState = SwerveModuleState.optimize(state, _io.getAngle());

    if (_desiredState.speedMetersPerSecond == 0 && !allowTurnInPlace) {
      _desiredState.angle = getModuleState().angle;
    }
    
    _isOpenLoop = isOpenLoop;
  }

  /** Updates this module periodically so it can reach the target state. */
  public void periodic() {    
    setDrive(_desiredState.speedMetersPerSecond);
    setAngle(_desiredState.angle);
  }

  /** Set the target velocity and acceleration for this module. */
  public void setDrive(double velocity) {
    _io.setVelocity(velocity, _isOpenLoop);
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
