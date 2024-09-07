package frc.lib;

import com.revrobotics.CANSparkMax;

public class REVUtil {
  /** 
   * Returns the name to use when displaying info about a CANSparkMax.
   */
  public static String getName(CANSparkMax spark) {
    return "CANSparkMax (" + spark.getDeviceId() + ")";
  }
}
