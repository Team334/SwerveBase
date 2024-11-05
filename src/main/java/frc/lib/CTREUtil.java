package frc.lib;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.lib.FaultsTable.FaultType;

/**
 * Utility class for CTRE devices.
 */
public class CTREUtil {
  /**
   * Attempts to configure a TalonFX. If config fails after multiple attempts, an error is reported to the fault logger.
   * 
   * @param talonFX The TalonFX to configure.
   * @param config The config to apply on the TalonFX.
   * 
   * @return True if the configuration fails.
   */
  public static boolean configure(TalonFX talonFX, TalonFXConfiguration config) {
    TalonFXConfigurator configurator = talonFX.getConfigurator();

    StatusCode statusCode = StatusCode.OK;

    for (int i = 0; i < 5; i++) {
      // attempts to apply the config to the talonfx, waits up to 50ms until the config is applied
      statusCode = configurator.apply(config);

      if (statusCode.isOK()) {
        break;
      }
    }

    String name = getName(talonFX);

    if (statusCode.isOK()) {
      FaultLogger.report(name + ": Config Apply Successful.", FaultType.INFO);
      return false;
    } 
    
    else {
      FaultLogger.report(name + ": Config Apply Failed: " + statusCode.getDescription(), FaultType.ERROR);
      return true;
    }
  }


  /**
   * Attempts to configure a CANcoder. If config fails after multiple attempts, an error is reported to the fault logger.
   * 
   * @param cancoder The CANcoder to configure.
   * @param config The config to apply on the CANcoder.
   * 
   * @return True if the configuration fails.
   */
  public static boolean configure(CANcoder cancoder, CANcoderConfiguration config) {
    CANcoderConfigurator configurator = cancoder.getConfigurator();

    StatusCode statusCode = StatusCode.OK;

    for (int i = 0; i < 5; i++) {
      // attempts to apply the config to the cancoder, waits up to 50ms until the config is applied
      statusCode = configurator.apply(config);

      if (statusCode.isOK()) {
        break;
      }
    }

    String name = getName(cancoder);

    if (statusCode.isOK()) {
      FaultLogger.report(name + ": Config Apply Successful.", FaultType.INFO);
      return false;
    } 
    
    else {
      FaultLogger.report(name + ": Config Apply Failed: " + statusCode.getDescription(), FaultType.ERROR);
      return true;
    }
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
