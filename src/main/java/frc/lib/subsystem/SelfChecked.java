package frc.lib.subsystem;

import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Alert.AlertType;

public interface SelfChecked {
  /**
   * Returns a Command that self checks this system.
   * 
   * @param alerter A consumer that alerts a given alert message with the specified alert type.
   * @param hasError A supplier that returns whether an error has occured somewhere in the self check.
   */
  public default Command selfCheck(BiConsumer<String, AlertType> alerter, BooleanSupplier hasError) {
    return Commands.none();
  };

  /**
   * Creates a new sequential command group that ends and prevents any other commands in the group
   * from running when a given condition is met.
   * 
   * This should be used to create a sequential command group of individual instant self check commands for robot self checking.
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
