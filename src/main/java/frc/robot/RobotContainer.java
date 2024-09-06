// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.Ports;
import frc.robot.subsystems.swerve.Swerve;
import monologue.Logged;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer implements Logged {
  private final Swerve _swerve = Swerve.create();

  private final CommandPS5Controller _driverController = new CommandPS5Controller(Ports.DRIVER_CONTROLLER);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    _swerve.setDefaultCommand(_swerve.drive(
      () -> -_driverController.getLeftY(),
      () -> -_driverController.getLeftX(),
      () -> -_driverController.getRightX()
    ));

    // Configure the trigger bindings
    configureBindings();

    // add self check command
    SmartDashboard.putData("Robot Self Check", Commands.sequence(
      new PrintCommand("Robot self check started."),
      _swerve.fullSelfCheck(),
      new PrintCommand("Robot self check finished.")
    ));
  }

  private void configureBindings() {
    
  }

  /**
   * @return The command to run in autonomous.
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new PrintCommand("AUTON!!!");
  }
}
