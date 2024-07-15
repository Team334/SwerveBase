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
  private Alert _statorCurrentLimitFaultAlert;
  private Alert _supplyCurrentLimitFaultAlert;

  // faults
  private StatusSignal<Boolean> _hardwareFault;
  private StatusSignal<Boolean> _forwardSoftLimitFault;
  private StatusSignal<Boolean> _reverseSoftLimitFault;
  private StatusSignal<Boolean> _statorCurrentLimitFault;
  private StatusSignal<Boolean> _supplyCurrentLimitFault;

  public SelfCheckedTalonFX(String name, TalonFX talon) {
    _hardwareFault = talon.getFault_Hardware();
    _forwardSoftLimitFault = talon.getFault_ForwardSoftLimit();
    _reverseSoftLimitFault = talon.getFault_ReverseSoftLimit();
    _statorCurrentLimitFault = talon.getFault_StatorCurrLimit();
    _supplyCurrentLimitFault = talon.getFault_SupplyCurrLimit();

    _hardwareFaultAlert = new Alert(name + ": Hardware fault active.", AlertType.ERROR);
    _forwardSoftLimitFaultAlert = new Alert(name + ": Forward soft limit reached.", AlertType.INFO);
    _reverseSoftLimitFaultAlert = new Alert(name + ": Reverse soft limit reached.", AlertType.INFO);
    _statorCurrentLimitFaultAlert = new Alert(name + ": Stator current limit reached.", AlertType.WARNING);
    _supplyCurrentLimitFaultAlert = new Alert(name + ": Supply current limit reached.", AlertType.WARNING);
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

    if (_statorCurrentLimitFault.getValue()) {
      _statorCurrentLimitFaultAlert.set(true);
    }

    if (_supplyCurrentLimitFault.getValue()) {
      _supplyCurrentLimitFaultAlert.set(true);
    }

    // refresh alerts and faults
    _hardwareFaultAlert.set(false);
    _forwardSoftLimitFaultAlert.set(false);
    _reverseSoftLimitFaultAlert.set(false);
    _statorCurrentLimitFaultAlert.set(false);
    _supplyCurrentLimitFaultAlert.set(false);

    BaseStatusSignal.refreshAll(
      _hardwareFault,
      _forwardSoftLimitFault,
      _reverseSoftLimitFault,
      _statorCurrentLimitFault,
      _supplyCurrentLimitFault
    );
  }
}
