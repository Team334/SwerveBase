package frc.lib;

import java.util.function.BiConsumer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Alert.AlertType;

public interface SelfChecked {
  /**
   * Returns a Command that self checks this system.
   * 
   * @param alert A consumer that alerts a given alert message with the specified alert type.
   */
  public Command selfCheck(BiConsumer<String, AlertType> alert);
}
