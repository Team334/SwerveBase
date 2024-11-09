package frc.lib;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.lib.FaultsTable.FaultType;

/**
 * Utility class for CTRE devices.
 */
public class CTREUtil {
  /**
   * Attempts to perform an action on a generic CTRE device. If the action returns a status code that is not OK, the 
   * action is re-attempted up to 5 times. If it fails after 5 attempts the status code is displayed in the fault logger.
   * 
   * @param action The action to perform (returns a StatusCode).
   * @param deviceName The name of the device for the fault logger.
   * 
   * @return True if the attempt failed, False otherwise.
   */
  public static boolean attempt(Supplier<StatusCode> action, String deviceName) {
    StatusCode statusCode = StatusCode.OK;

    for (int i = 0; i < 5; i++) {
      // performs the action
      statusCode = action.get();

      if (statusCode.isOK()) {
        break;
      }
    }

    // successful attempt
    if (statusCode.isOK()) {
      FaultLogger.report(deviceName + ": Config Apply Successful.", FaultType.INFO);
      return false;
    } 
    
    // failed attempt
    else {
      FaultLogger.report(deviceName + ": Config Apply Failed: " + statusCode.getDescription(), FaultType.ERROR);
      return true;
    }
  }

  /**
   * Attempts to perform an action on a TalonFX. If the action returns a status code that is not OK, the 
   * action is re-attempted up to 5 times. If it fails after 5 attempts the status code is displayed in the fault logger.
   * 
   * @param action The action to perform (returns a StatusCode).
   * @param talonFX The TalonFX for the fault logger.
   * 
   * @return True if the attempt failed, False otherwise.
   */
  public static boolean attempt(Supplier<StatusCode> action, TalonFX talonFX) {
    return attempt(action, getName(talonFX));
  }

  /**
   * Attempts to perform an action on a CANCoder. If the action returns a status code that is not OK, the 
   * action is re-attempted up to 5 times. If it fails after 5 attempts the status code is displayed in the fault logger.
   * 
   * @param action The action to perform (returns a StatusCode).
   * @param cancoder The cancoder for the fault logger.
   * 
   * @return True if the attempt failed, False otherwise.
   */
  public static boolean attempt(Supplier<StatusCode> action, CANcoder cancoder) {
    return attempt(action, getName(cancoder));
  }

  /** 
   * Returns the name to use when displaying info about a TalonFX.
   */
  public static String getName(TalonFX talonFX) {
    return "TalonFX (" + talonFX.getDeviceID() + ")";
  }

  /**
   * Returns the name to use when displaying info about a CANcoder.
   */
  public static String getName(CANcoder cancoder) {
    return "CANcoder (" + cancoder.getDeviceID() + ")";
  }
}
