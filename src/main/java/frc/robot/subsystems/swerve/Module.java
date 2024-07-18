package frc.robot.subsystems.swerve;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Robot;
import monologue.Annotations.Log;

public class Module {
  private final ModuleIO _io;

  private double _oldVelocity = 0; 

  @Log.NT
  private SwerveModuleState _desiredState = new SwerveModuleState();

  public Module(ModuleIO io) {
    _io = io;
  }

  /** Get the measured state of this module. */
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
}
