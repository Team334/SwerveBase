package frc.robot.subsystems.swerve.gyros;

import com.ctre.phoenix6.BaseStatusSignal;

import edu.wpi.first.math.geometry.Rotation2d;

/** NavX Gyro Implementation. */
public class NavXGyro implements GyroIO {
  @Override
  public Rotation2d getYaw() {
    return new Rotation2d();
  }

  @Override
  public BaseStatusSignal getOdomSignal() {
    return null;
  }
}
