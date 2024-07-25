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
  private boolean _hasErrors = false;

  /** Clears this subsystem's alerts. */
  protected void clearAlerts() {
    _alerts.forEach(a -> a.remove());
    _alerts.clear();
  }

  /** Alerts a new message under this subsystem. */
  protected void alert(String message, AlertType alertType) {
    Alert alert = new Alert(getName() + " Alerts", message, alertType);
    alert.set(true);
    _alerts.add(alert);

    if (alertType == AlertType.ERROR) _hasErrors = true; 
  }

  /** Returns whether this subsystem has alerted errors. */
  public boolean hasErrors() {
    return _hasErrors;
  }

  /** Returns a full Command that self checks this Subsystem for pre-match. */
  public Command fullSelfCheck() {
    Command selfCheck = Commands.sequence(
      Commands.runOnce(this::clearAlerts),
      selfCheck(this::alert, this::hasErrors).until(this::hasErrors),
      Commands.runOnce(() -> {
        if (_hasErrors) { 
          alert(getName() + ": Self check failed, check errors!", AlertType.ERROR);
        } else {
          alert(getName() + ": Self check finished.", AlertType.INFO);
        }
      })
    );

    selfCheck.addRequirements(this);

    return selfCheck;
  }
}
