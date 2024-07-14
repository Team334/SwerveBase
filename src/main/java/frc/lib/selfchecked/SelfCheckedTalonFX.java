package frc.lib.selfchecked;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.lib.Alert;
import frc.lib.Alert.AlertType;

public class SelfCheckedTalonFX extends SelfChecked {
  // TODO: add more faults to check

  // alerts
  private Alert _hardwareFaultAlert;
  private Alert _forwardSoftLimitFaultAlert;
  private Alert _reverseSoftLimitFaultAlert;

  // faults
  private StatusSignal<Boolean> _hardwareFault;
  private StatusSignal<Boolean> _forwardSoftLimitFault;
  private StatusSignal<Boolean> _reverseSoftLimitFault;

  public SelfCheckedTalonFX(String name, TalonFX talon) {
    _hardwareFault = talon.getFault_Hardware();
    _forwardSoftLimitFault = talon.getFault_ForwardSoftLimit();
    _reverseSoftLimitFault = talon.getFault_ReverseSoftLimit();

    _hardwareFaultAlert = new Alert(name + ": Hardware fault active", AlertType.ERROR);
    _forwardSoftLimitFaultAlert = new Alert(name + ": Forward soft limit reached", AlertType.WARNING);
    _reverseSoftLimitFaultAlert = new Alert(name + ": Reverse soft limit reached", AlertType.WARNING); 
  }

  @Override
  public void checkFaults() {
    if (_hardwareFault.getValue()) {
      _hardwareFaultAlert.set(true);
    }

    if (_forwardSoftLimitFault.getValue()) {
      _forwardSoftLimitFaultAlert.set(true);
    }

    if (_reverseSoftLimitFault.getValue()) {
      _reverseSoftLimitFaultAlert.set(true);
    }

    // refresh alerts and faults
    _hardwareFaultAlert.set(false);
    _forwardSoftLimitFaultAlert.set(false);
    _reverseSoftLimitFaultAlert.set(false);

    BaseStatusSignal.refreshAll(
      _hardwareFault,
      _forwardSoftLimitFault,
      _reverseSoftLimitFault
    );
  }
}
