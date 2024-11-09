// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/** Prints and logs a given message (DataLogManager.log in a command). */
public class PrintAndLog extends InstantCommand {
  /**
   * Prints and logs the given message.
   */
  public PrintAndLog(String message) {
    super(() -> {
      DataLogManager.log(message);
    });
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
