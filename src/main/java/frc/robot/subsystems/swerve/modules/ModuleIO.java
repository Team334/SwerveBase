package frc.robot.subsystems.swerve.modules;

import com.ctre.phoenix6.BaseStatusSignal;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.subsystem.SelfChecked;

public interface ModuleIO extends SelfChecked {
  /** Give the distinct module name to the IO. */
  public void setName(String moduleName);

  /** Returns the velocity of the drive wheel in m/s. */
  public double getVelocity();

  /**
   * Sets the targets of the drive motor.
   * 
   * @param velocity The target velocity in m/s.
   * @param isOpenLoop Whether the target velocity is to be meet through open loop control or not.
   */
  public void setVelocity(double velocity, boolean isOpenLoop);

  /** Returns the angle of the module. */
  public Rotation2d getAngle();

  /** Sets the angle of the module. */
  public void setAngle(Rotation2d angle);

  /**
   * Returns the drive motor position (distance traveled).
  */
  public double getPosition();

  /**
   * Returns the CTRE status signals to be refreshed periodically in the odom thread. In sim this returns
   * and empty array.
   * 
   * <pre>
   * array[0] - Drive Velocity
   * array[1] - Turn Angle
   * array[2] - Drive Position
   * </pre>
   */
  public BaseStatusSignal[] getOdomSignals();

  /** Sets the voltage of the drive motor. (ONLY NECESSARY FOR SYSID) */
  public void setDriveVoltage(double volts);

  /** Sets the voltage of the turn motor. (ONLY NECESSARY FOR SYSID) */
  public void setTurnVoltage(double volts);
}
