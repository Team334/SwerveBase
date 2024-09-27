// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.lib.InputStream;
import frc.lib.PrintAndLog;
import frc.robot.Constants.Ports;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.Autos;
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
      InputStream.of(_driverController::getLeftY)
        .negate()
        .scale(SwerveConstants.MAX_TRANSLATIONAL_SPEED.in(MetersPerSecond)),
      InputStream.of(_driverController::getLeftX)
        .negate()
        .scale(SwerveConstants.MAX_TRANSLATIONAL_SPEED.in(MetersPerSecond)),
      InputStream.of(_driverController::getRightX)
        .negate()
        .scale(SwerveConstants.MAX_ANGULAR_SPEED.in(RadiansPerSecond))
    ));

    // Configure the trigger bindings
    configureBindings();

    SmartDashboard.putData("Swerve Self-Check", _swerve.fullSelfCheck());

    // add self check command
    SmartDashboard.putData(
      "Run Self-Check",
      Commands.sequence(
        new PrintAndLog("Robot self check started."),
        _swerve.fullSelfCheck(),
        new PrintAndLog("Robot self check finished.")
      ).onlyIf(() -> DriverStation.isTestEnabled())
       .withName("Robot Self-Check")
    );
  }

  private void configureBindings() {
    _driverController.cross().whileTrue(_swerve.brake());
    _driverController.circle().onTrue(_swerve.toggleOriented());
  }

  /**
   * @return The command to run in autonomous.
   */
  public Command getAutonomousCommand() {
    return Autos.none();
  }
}
