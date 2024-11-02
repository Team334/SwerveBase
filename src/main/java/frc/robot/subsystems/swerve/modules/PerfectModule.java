package frc.robot.subsystems.swerve.modules;

import com.ctre.phoenix6.BaseStatusSignal;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;

/**
 * Represents a simulated perfect module. For the module, currentState = desiredState right away.
 */
public class PerfectModule implements ModuleIO {
  private double _driveVelocity = 0;
  private double _drivePosition = 0;

  private Rotation2d _turnAngle = new Rotation2d(); 

  @Override
  public void setName(String moduleName) {}

  @Override
  public double getVelocity() {
    return _driveVelocity;
  }

  @Override
  public void setVelocity(double velocity, boolean isOpenLoop) {
    _driveVelocity = velocity;
    _drivePosition += velocity * Robot.kDefaultPeriod;
  }

  @Override
  public Rotation2d getAngle() {
    return _turnAngle;
  }

  @Override
  public void setAngle(Rotation2d angle) {
    _turnAngle = angle;
  }

  @Override
  public double getPosition() {
    return _drivePosition;
  }

  @Override
  public BaseStatusSignal[] getOdomSignals() {
    return new BaseStatusSignal[]{};
  }

  @Override
  public void setDriveVoltage(double volts) {}

  @Override
  public void setTurnVoltage(double volts) {}
}
