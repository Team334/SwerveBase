package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ModuleIO {
  /**
   * Sets the targets of the drive motor.
   * @param velocity The target velocity in m/s.
   * @param acceleration The target acceleration in m/s^2.
   */
  public void setDriveTargets(double velocity, double acceleration);

  /** Sets the angle of the module in degrees. */
  public void setTurnAngle(double angle);

  /** Returns the velocity of the drive wheel in m/s. */
  public double getDriveVelocity();

  /** Returns the angle of the module in degrees. */
  public Rotation2d getTurnPosition();
}
