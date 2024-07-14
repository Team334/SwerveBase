package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

/**
 * Devices can be added to this alerter to have their faults periodically alerted.
 */
public class Alerter {
  private static List<Runnable> _alerters = new ArrayList<Runnable>();
  
  // TODO: Create all "add" functions here

  /**
   * Checks all added devices and alerts their faults.
   */
  public static void alert() {
    for (Runnable alerter : _alerters) {
      alerter.run();
    }
  }
}
