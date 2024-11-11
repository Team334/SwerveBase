package frc.robot.subsystems.swerve.gyros;

import com.ctre.phoenix6.BaseStatusSignal;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;

/** NavX Gyro Implementation. */
public class NavXGyro implements GyroIO {
  private final AHRS _ahrs = new AHRS();

  @Override
  public Rotation2d getYaw() {
    return _ahrs.getRotation2d();
  }

  @Override
  public boolean isConnected() {
    // Unlike CAN devices, the reason for why the navx wouldn't send data is if it got disconnected from the 
    // rio or if it's broken, either way the navx would be done for the rest of the match.
    return _ahrs.isConnected();
  }

  @Override
  public BaseStatusSignal getOdomSignal() {
    return null;
  }
}
