// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
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
  // TODO: tune all these values according to robot
  public static class Ports {
    public static final int DRIVER_CONTROLLER = 0;
  }

  public static class SwerveConstants {
    public static final double ODOM_FREQUENCY = 100;

    public static final Measure<Velocity<Distance>> MAX_TRANSLATIONAL_SPEED = FeetPerSecond.of(16.5);
    public static final Measure<Velocity<Velocity<Distance>>> MAX_TRANSLATIONAL_ACCELERATION = FeetPerSecond.of(18).per(Second);
    
    public static final Measure<Velocity<Angle>> MAX_ANGULAR_SPEED = RadiansPerSecond.of(Math.PI);
    public static final Measure<Velocity<Velocity<Angle>>> MAX_ANGULAR_ACCELERATION = RadiansPerSecond.of(Math.PI * 1.5).per(Second);

    public static final Measure<Distance> DRIVE_WIDTH = Meters.of(0.584);
    public static final Measure<Distance> DRIVE_LENGTH = Meters.of(0.584);

    public static final Translation2d[] MODULE_POSITIONS = new Translation2d[]{
      new Translation2d(DRIVE_LENGTH.divide(2), DRIVE_WIDTH.divide(2)),
      new Translation2d(DRIVE_LENGTH.divide(2), DRIVE_WIDTH.divide(2).negate()),
      new Translation2d(DRIVE_LENGTH.divide(2).negate(), DRIVE_WIDTH.divide(2).negate()),
      new Translation2d(DRIVE_LENGTH.divide(2).negate(), DRIVE_WIDTH.divide(2))
    };
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

    // feedforward (meters)
    public static final double DRIVE_KS = 0.1;
    public static final double DRIVE_KV = 2.38;
    public static final double DRIVE_KA = 0.01;

    public static final double DRIVE_KP = 0;

    public static final double DRIVE_GEARING = 6.75;

    public static final Measure<Distance> DRIVE_WHEEL_CIRCUMFERENCE = Meters.of(0.05 * 2 * Math.PI); 
  
    // turn feedforward is only needed for sim (radians)
    public static final double TURN_KV = 1.58;
    public static final double TURN_KA = 0.001;

    public static final double TURN_KP = 1.5;

    public static final double TURN_GEARING = 150/7;
  }
}
