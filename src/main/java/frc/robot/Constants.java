// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Velocity;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

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

  public static class SwerveConstants {
    // (rate of change of distance)
    public static final Measure<Velocity<Distance>> MAX_TRANSLATIONAL_SPEED = MetersPerSecond.of(0);
    // (rate of change of rate of change of distance)
    public static final Measure<Velocity<Velocity<Distance>>> MAX_TRANSLATIONAL_ACCELERATION = MetersPerSecondPerSecond.of(0);
    
    public static final Measure<Velocity<Angle>> MAX_ANGULAR_SPEED = RadiansPerSecond.of(0);
    public static final Measure<Velocity<Velocity<Angle>>> MAX_ANGULAR_ACCELERATION = RadiansPerSecond.of(0).per(Second);


    public static final Translation2d[] WHEEL_LOCATIONS = {}; 
  }

  public static class SwerveModuleConstants {
    public static final int FRONT_LEFT_DRIVE_ID = 0;
    public static final int FRONT_LEFT_TURN_ID = 0;
    public static final int FRONT_LEFT_ENCODER_ID = 0; 

    public static final int FRONT_RIGHT_DRIVE_ID = 0;
    public static final int FRONT_RIGHT_TURN_ID = 0;
    public static final int FRONT_RIGHT_ENCODER_ID = 0; 

    public static final int BACK_RIGHT_DRIVE_ID = 0;
    public static final int BACK_RIGHT_TURN_ID = 0;
    public static final int BACK_RIGHT_ENCODER_ID = 0; 

    public static final int BACK_LEFT_DRIVE_ID = 0;
    public static final int BACK_LEFT_TURN_ID = 0;
    public static final int BACK_LEFT_ENCODER_ID = 0; 

    public static final double DRIVE_KS = 1;
    public static final double DRIVE_KV = 1;
    public static final double DRIVE_KA = 1;

    public static final double DRIVE_KP = 1;

    public static final double DRIVE_GEARING = 6.75;

    public static final Measure<Distance> DRIVE_WHEEL_CIRCUMFERENCE = Meters.of(0.05 * 2 * Math.PI); 
  
    // this is only needed for sim
    public static final double TURN_KV = 1;
    public static final double TURN_KA = 1;

    public static final double TURN_KP = 1;

    public static final double TURN_GEARING = 150/7;
  }
}
