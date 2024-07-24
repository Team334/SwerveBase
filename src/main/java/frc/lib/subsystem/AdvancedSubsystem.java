package frc.lib.subsystem;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Alert;
import frc.lib.Alert.AlertType;

public abstract class AdvancedSubsystem extends SubsystemBase implements SelfChecked {
  private Set<Alert> _alerts = new HashSet<Alert>();
  
  /** Clears this subsystem's alerts. */
  protected void clearAlerts() {
    _alerts.forEach(a -> a.remove());
  }

  /** Alerts a new message under this subsystem. */
  protected void alert(String message, AlertType alertType) {
    new Alert(getName() + " Alerts", message, alertType).set(true);
  }

  /** Returns a full Command that self checks this Subsystem for pre-match. */
  public Command fullSelfCheck() {
    Command selfCheck = Commands.sequence(
      Commands.runOnce(this::clearAlerts),
      selfCheck(this::alert)
    );

    selfCheck.addRequirements(this);

    return selfCheck;
  }
}
