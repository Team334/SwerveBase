package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ModuleIO {
  /** Returns the position of the drive wheel in meters. */
  public double getDrivePosition();

  /** Returns the velocity of the drive wheel in m/s. */
  public double getDriveVelocity();

  /**
   * Sets the targets of the drive motor.
   * @param velocity The target velocity in m/s.
   * @param acceleration The target acceleration in m/s^2.
   */
  public void setDriveTargets(double velocity, double acceleration);

  /** Returns the angle of the module in degrees. */
  public Rotation2d getAngle();

  /** Sets the angle of the module in degrees. */
  public void setAngle(double angle);
}
