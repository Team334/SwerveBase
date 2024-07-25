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
}
