package frc.robot.subsystems.swerve.gyros;

import com.ctre.phoenix6.BaseStatusSignal;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.subsystem.SelfChecked;

public interface GyroIO extends SelfChecked {
  /** The yaw of the gyro. */
  public Rotation2d getYaw();

  /** Returns the CTRE StatusSignal for yaw on this device. In sim/navx this returns null. */
  public BaseStatusSignal getOdomSignal();
}
