package frc.robot.subsystems.swerve;

public interface ModuleIO {
  /**
   * Sets the targets of the drive motor.
   * @param velocity The target velocity in m/s.
   * @param acceleration The target acceleration in m/s^2.
   */
  public void setDriveTargets(double velocity, double acceleration);

  /** Sets the angle of the turn motor in degrees. */
  public void setTurnAngle(double angle);

  /** Returns the velocity of the drive motor in m/s. */
  public double getDriveVelocity();

  /** Returns the position of the rotation motor in degrees. */
  public double getTurnPosition();
}
