package frc.lib.subsystem;

import java.util.function.BiConsumer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Alert.AlertType;

public interface SelfChecked {
  /**
   * Returns a Command that self checks this system.
   * 
   * @param alerter A consumer that alerts a given alert message with the specified alert type.
   */
  public default Command selfCheck(BiConsumer<String, AlertType> alerter) {
    return Commands.none();
  };
}
