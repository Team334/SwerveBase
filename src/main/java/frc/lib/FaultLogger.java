package frc.lib;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.lib.selfchecked.SelfChecked;
import frc.lib.selfchecked.SelfCheckedTalonFX;

/**
 * Devices are added to this fault logger to have their faults alerted and logged.
 */
public class FaultLogger {
  private static List<SelfChecked> _devices = new ArrayList<SelfChecked>();
  
  /** Register a new SelfChecked device to the FaultLogger. */
  public static void register(SelfChecked device) {
    _devices.add(device);
  }

  /** Register a new TalonFX to the FaultLogger. */
  public static void register(String name, TalonFX talonFX) {
    register(new SelfCheckedTalonFX(name, talonFX));
  }

  /** Checks all added devices and alerts their faults. */
  public static void checkAllFaults() {
    for (SelfChecked device : _devices) {
      device.checkFaults();
    }
  }
}
