package frc.lib.util;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class CTREUtil {
  /**
   * Attempts to configure a TalonFX. If config fails after multiple attempts, an error is reported to the fault logger.
   * 
   * @param talonFX The TalonFX to configure.
   * @param config The config to apply on the TalonFX.
   * 
   * @return Whether the configuration was successful or not.
   */
  public static boolean configure(TalonFX talonFX, TalonFXConfiguration config) {
    // TODO: add code here
    return true;
  }

  /** 
   * Returns the name to use when displaying info about a TalonFX.
   */
  public static String getName(TalonFX talonFX) {
    return "TalonFX (" + talonFX.getDeviceID() + ")";
  }
}
