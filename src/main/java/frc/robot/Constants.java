// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import static edu.wpi.first.units.Units.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class Ports {
    public static final int DRIVER_CONTROLLER = 0;
  }

  public static class SwerveModule {
    public static final double DRIVE_KS = 0;
    public static final double DRIVE_KV = 0;
    public static final double DRIVE_KA = 0;

    public static final double DRIVE_KP = 0;

    public static final double DRIVE_GEARING = 6.75;

    public static final Measure<Distance> DRIVE_WHEEL_CIRCUMFERENCE = Meters.of(0.05 * 2 * Math.PI); 
  
    // this is only needed for sim
    public static final double TURN_KV = 0;
    public static final double TURN_KA = 0;

    public static final double TURN_KP = 0;

    public static final double TURN_GEARING = 150/7;
  }
}
