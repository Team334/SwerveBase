package frc.robot.subsystems.swerve.gyros;

import com.ctre.phoenix6.BaseStatusSignal;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.subsystem.SelfChecked;

public interface GyroIO extends SelfChecked {
  /** The yaw of the gyro. */
  public Rotation2d getYaw();

  /** Returns whether the gyro is connected to the rio. This is only relevant for the navx. */
  public boolean isConnected();

  /** Returns the CTRE StatusSignal for yaw on this device. In sim/navx this returns null. */
  public BaseStatusSignal getOdomSignal();
}
