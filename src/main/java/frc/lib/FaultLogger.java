package frc.lib;

import java.util.ArrayList;
import java.util.List;

import frc.lib.selfchecked.SelfChecked;

/**
 * Devices are added to this fault logger to have their faults alerted and logged.
 */
public class FaultLogger {
  private static List<SelfChecked> _devices = new ArrayList<SelfChecked>();
  
  // TODO: Create all "add" functions here

  /**
   * Checks all added devices and alerts their faults.
   */
  public static void checkAllFaults() {
    for (SelfChecked device : _devices) {
      device.checkFaults();
    }
  }
}
