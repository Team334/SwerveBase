package frc.robot.subsystems.swerve.gyros;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.subsystem.SelfChecked;

public interface GyroIO extends SelfChecked {
  /** The yaw of the gyro. */
  public Rotation2d getYaw();
}
