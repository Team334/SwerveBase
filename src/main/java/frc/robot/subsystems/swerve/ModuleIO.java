package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.subsystem.SelfChecked;

public interface ModuleIO extends SelfChecked {
  /** Returns the velocity of the drive wheel in m/s. */
  public double getDriveVelocity();

  /**
   * Sets the targets of the drive motor.
   * @param velocity The target velocity in m/s.
   * @param isOpenLoop Whether the target velocity is to be meet through open loop control or not.
   */
  public void setVelocity(double velocity, boolean isOpenLoop);

  /** Returns the angle of the module. */
  public Rotation2d getAngle();

  /** Sets the angle of the module. */
  public void setAngle(Rotation2d angle);

  /**
   * Returns the drive motor position.
  */
  public double getDrivePosition();
}
