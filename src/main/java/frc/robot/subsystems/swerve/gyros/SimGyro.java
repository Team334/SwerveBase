package frc.robot.subsystems.swerve.gyros;

import edu.wpi.first.math.geometry.Rotation2d;

public class SimGyro implements GyroIO {
  @Override
  public Rotation2d getYaw() {
    return new Rotation2d();
  }
}