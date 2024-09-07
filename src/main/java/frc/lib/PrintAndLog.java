// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PrintAndLog extends InstantCommand {
  /**
   * Prints and logs the given message.
   */
  public PrintAndLog(String message) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(() -> {
      System.out.println(message);
      DataLogManager.log(message);
    });
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
