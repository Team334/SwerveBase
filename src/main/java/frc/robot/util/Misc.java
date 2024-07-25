package frc.robot.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** Any miscellaneous static util functions are here. */
public class Misc {
  /**
   * Creates a new sequential command group that ends and prevents any other commands in the group
   * from running when a given condition is met.
   * 
   * (This is needed due to issues in wpilib's sequential command group)
   * 
   * @param condition The condition that if true, ends the command.
   * @param commands The commands that make up the group.
   */
  public static Command sequentialUntil(BooleanSupplier condition, Command... commands) {
    for (int i = 1; i < commands.length; ++i) {
      commands[i] = commands[i].unless(condition);
    }
    return Commands.sequence(commands).until(condition);
  }
}
