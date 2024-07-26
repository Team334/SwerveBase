// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.Swerve;

public class TeleopDrive extends Command {
  private final Swerve _swerve;

  private final SlewRateLimiter _xVelLimiter = new SlewRateLimiter(SwerveConstants.MAX_TRANSLATIONAL_ACCELERATION.magnitude());
  private final SlewRateLimiter _yVelLimiter = new SlewRateLimiter(SwerveConstants.MAX_TRANSLATIONAL_ACCELERATION.magnitude());
  private final SlewRateLimiter _omegaVelLimiter = new SlewRateLimiter(SwerveConstants.MAX_ANGULAR_ACCELERATION.magnitude());

  private final DoubleSupplier _velX;
  private final DoubleSupplier _velY;
  private final DoubleSupplier _velOmega;

  /** Creates a new TeleopDrive. */
  public TeleopDrive(Swerve swerve, DoubleSupplier velX, DoubleSupplier velY, DoubleSupplier velOmega) {
    _swerve = swerve;

    _velX = velX;
    _velY = velY;
    _velOmega = velOmega;

    addRequirements(_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _swerve.drive(
      _xVelLimiter.calculate(_velX.getAsDouble() * SwerveConstants.MAX_TRANSLATIONAL_SPEED.magnitude()),
      _yVelLimiter.calculate(_velY.getAsDouble() * SwerveConstants.MAX_TRANSLATIONAL_SPEED.magnitude()),
      _omegaVelLimiter.calculate(_velOmega.getAsDouble() * SwerveConstants.MAX_ANGULAR_SPEED.magnitude())
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
