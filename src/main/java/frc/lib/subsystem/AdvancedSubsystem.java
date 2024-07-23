package frc.lib.subsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Alert;
import frc.lib.Alert.AlertType;

public abstract class AdvancedSubsystem extends SubsystemBase implements SelfChecked {
  /** Alerts a new message under this subsystem. */
  protected void alert(String message, AlertType alertType) {
    new Alert(getName() + " Alerts", message, alertType).set(true);
  }

  /** Returns a full Command that self checks this Subsystem for pre-match. */
  public Command fullSelfCheck() {
    return Commands.sequence(
      selfCheck(this::alert)
    );
  }
}
