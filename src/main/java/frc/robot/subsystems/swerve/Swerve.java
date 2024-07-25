// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import java.util.function.BiConsumer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Alert.AlertType;
import frc.lib.subsystem.AdvancedSubsystem;
import monologue.Logged;

public class Swerve extends AdvancedSubsystem implements Logged {
  private final SwerveModule _m = new SwerveModule(new RealModule("front left", 1, 1, 1)); 

  /** Creates a new Swerve. */
  public Swerve() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public Command selfCheck(BiConsumer<String, AlertType> alerter) {
    return Commands.sequence(
      _m.selfCheck(alerter)
    );
  }
}
