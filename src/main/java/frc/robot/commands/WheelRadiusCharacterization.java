// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;

// TODO
/**
 * Finds the radius of the drive wheels.
 */
public class WheelRadiusCharacterization extends Command {
  private final Swerve _swerve;
  
  /** Creates a new WheelRadiusCharacterization. */
  public WheelRadiusCharacterization(Swerve swerve) {
    _swerve = swerve;
    addRequirements(_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
