package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Meters;
import static frc.lib.subsystem.SelfChecked.sequentialUntil;

import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.BaseStatusSignal;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.FaultsTable.FaultType;
import frc.lib.subsystem.SelfChecked;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.swerve.modules.ModuleIO;

public class SwerveModule implements SelfChecked {
  private final String _name;
  private final ModuleIO _io;

  private SwerveModuleState _desiredState = new SwerveModuleState();

  private boolean _isOpenLoop = false;

  public SwerveModule(String name, ModuleIO io) {
    _io = io;
    _name = name;
    
    _io.setName(_name);
  }

  /** The distinct name of this module based on where it is on the chassis. */
  public String getName() {
    return _name;
  }

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
  public BaseStatusSignal[] getOdomSignals() {
    return _io.getOdomSignals();
  }

  /** Returns the last desired set state for this module. */
  public SwerveModuleState getDesiredState() {
    return _desiredState;
  }

  /** Get the measured state of this module. */
  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(
      Units.radiansToRotations(_io.getVelocity()) * ModuleConstants.DRIVE_CIRCUMFERENCE.in(Meters), 
      _io.getAngle()
    );
  }

  /**
   * Returns the position (distance, angle) of this module. This method
   * should only be called inside the odom thread to avoid thread safety issues,
   * or enclosed in a lock shared by the odom thread.
   */
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(
      Units.radiansToRotations(_io.getPosition()) * ModuleConstants.DRIVE_CIRCUMFERENCE.in(Meters), 
      _io.getAngle()
    );
  }

  /** 
   * Set the desired target state for this module.
   * 
   * @param state The target state.
   * @param isOpenLoop Whether the velocity of the state is met in an open loop (FF only) or closed loop manner (FF + PID).
   */
  public void setModuleState(SwerveModuleState state, boolean isOpenLoop) {
    // angle optimization handled by swerve setpoint generator
    _desiredState = state;
    
    // _desiredState.speedMetersPerSecond *= _desiredState.angle.minus(_io.getAngle()).getCos();

    _isOpenLoop = isOpenLoop;
  }

  /** Updates this module periodically so it can reach the target state. */
  public void periodic() {
    setDrive(_desiredState.speedMetersPerSecond);
    setAngle(_desiredState.angle);
  }

  /** Set the target velocity and acceleration for this module in m/s. */
  public void setDrive(double velocity) {
    _io.setVelocity(
      Units.rotationsToRadians(velocity / ModuleConstants.DRIVE_CIRCUMFERENCE.in(Meters)),
      _isOpenLoop
    );
  }

  /** Set the target angle for this module. */ 
  public void setAngle(Rotation2d angle) {
    _io.setAngle(angle);
  }

  /** Sets the voltage of the drive motor, while keeping the turn motor at 0 degrees. (ONLY NECESSARY FOR SYSID) */
  public void setDriveVoltage(double volts) {
    _io.setDriveVoltage(volts);
    _io.setAngle(new Rotation2d());
  }

  /** Sets the voltage of the turn motor, while keeping the drive motor at zero volts. (ONLY NECESSARY FOR SYSID) */
  public void setTurnVoltage(double volts) {
    _io.setTurnVoltage(volts);
    _io.setDriveVoltage(0);
  }

  @Override
  public Command selfCheck(BiConsumer<String, FaultType> faultAdder, BooleanSupplier hasError) {
    return sequentialUntil(
      hasError,
      _io.selfCheck(faultAdder, hasError)
    );
  }
}
