package frc.robot.subsystems.swerve;

import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Alert.AlertType;
import frc.lib.subsystem.SelfChecked;
import frc.robot.Robot;
import monologue.Logged;
import monologue.Annotations.Log;

public class SwerveModule implements SelfChecked, Logged {
  private final ModuleIO _io;

  private final String _name;

  private double _oldVelocity = 0; 

  private SwerveModuleState _desiredState = new SwerveModuleState();

  public SwerveModule(String name, ModuleIO io) {
    _io = io;
    _name = name;
    
    if (_io instanceof RealModule) ((RealModule) _io).setName(_name);
  }

  /** Returns the last desired set state for this module. */
  @Log.NT
  public SwerveModuleState getDesiredState() {
    return _desiredState;
  }

  /** Get the measured state of this module. */
  @Log.NT
  public SwerveModuleState getState() {
    return new SwerveModuleState(_io.getDriveVelocity(), _io.getAngle());
  }

  /** Set the target state for this module. */
  public void setState(SwerveModuleState state) {
    _desiredState = SwerveModuleState.optimize(state, _io.getAngle());

    setVelocity(_desiredState.speedMetersPerSecond);
    setAngle(_desiredState.angle.getDegrees());
  }

  /** Set the target velocity for this module. */
  public void setVelocity(double velocity) {
    double acceleration = (velocity - _oldVelocity) / Robot.kDefaultPeriod;
    _io.setDriveTargets(velocity, acceleration);
  }

  /** Set the target angle for this module. */ 
  public void setAngle(double angle) {
    _io.setAngle(angle);
  }

  @Override
  public Command selfCheck(BiConsumer<String, AlertType> alerter, BooleanSupplier hasError) {
    return _io.selfCheck(alerter, hasError);
  }
}
