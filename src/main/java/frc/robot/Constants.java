// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Per;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import frc.robot.util.VisionPoseEstimator.VisionPoseEstimatorConstants;

import static edu.wpi.first.units.Units.*;

import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;

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
  public static final String SIM_SYSID_LOG_PREFIX = "Sim SysId/";

  public static class Ports {
    public static final int DRIVER_CONTROLLER = 0;
  }

  public static class CAN {
    public static final boolean REDUCE_CONTROL_LATENCY = true;
  }

  public static final class VisionConstants {
    public static final Vector<N3> SINGLE_TAG_BASE_STDDEVS = VecBuilder.fill(5, 5, 5);
    public static final Vector<N3> MULTI_TAG_BASE_STDDEVS = VecBuilder.fill(1, 1, 1);

    public static final double X_BOUND_MARGIN = 0;
    public static final double Y_BOUND_MARGIN = 0;
    public static final double Z_BOUND_MARGIN = 0.1;

    public static final List<VisionPoseEstimatorConstants> CAM_CONSTANTS = List.of(
      new VisionPoseEstimatorConstants(
        "left-arducam",
        new Transform3d(new Translation3d(0, 0, 1), new Rotation3d()),
        0.2,
        0.0001
      )
    );
  }

  public static final class FieldConstants {
    public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    public static final int FIELD_TAG_AMOUNT = FIELD_LAYOUT.getTags().size();
  }

  public static class SwerveConstants {
    public static final double ODOM_FREQUENCY = 100;

    public static final int PIGEON_ID = 0;

    public static final Measure<Distance> DRIVE_WIDTH = Meters.of(0.584);
    public static final Measure<Distance> DRIVE_LENGTH = Meters.of(0.584);

    public static final Measure<Distance> DRIVE_RADIUS = Meters.of(
      Math.sqrt(Math.pow(DRIVE_WIDTH.in(Meters), 2) + Math.pow(DRIVE_LENGTH.in(Meters), 2))
    );

    public static final Measure<Velocity<Distance>> MAX_TRANSLATIONAL_SPEED = MetersPerSecond.of(4.98);
    public static final Measure<Velocity<Velocity<Distance>>> MAX_TRANSLATIONAL_ACCELERATION = MetersPerSecondPerSecond.of(63.84);
    
    public static final Measure<Velocity<Angle>> MAX_ANGULAR_SPEED = RadiansPerSecond.of(
      MAX_TRANSLATIONAL_SPEED.divide(DRIVE_RADIUS.in(Meters)).in(MetersPerSecond)
    );
    
    public static final Measure<Velocity<Velocity<Angle>>> MAX_ANGULAR_ACCELERATION = RadiansPerSecond.per(Second).of(
      MAX_TRANSLATIONAL_ACCELERATION.divide(DRIVE_RADIUS.in(Meters)).in(MetersPerSecondPerSecond)
    );
    
    public static final Translation2d[] MODULE_POSITIONS = new Translation2d[]{
      new Translation2d(DRIVE_LENGTH.divide(2), DRIVE_WIDTH.divide(2)),
      new Translation2d(DRIVE_LENGTH.divide(2), DRIVE_WIDTH.divide(2).negate()),
      new Translation2d(DRIVE_LENGTH.divide(2).negate(), DRIVE_WIDTH.divide(2).negate()),
      new Translation2d(DRIVE_LENGTH.divide(2).negate(), DRIVE_WIDTH.divide(2))
    };
  }

  public static class ModuleConstants {
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

    public static final Measure<Current> DRIVE_STATOR_CURRENT_LIMIT = Amps.of(60);
    public static final Measure<Current> SUPPLY_CURRENT_LIMIT = Amps.of(30);
    
    // feedforward
    public static final Measure<Voltage> DRIVE_KS = Volts.of(0);
    public static final Measure<Per<Voltage, Velocity<Angle>>> DRIVE_KV = VoltsPerRadianPerSecond.of(0.12053);
    public static final Measure<Per<Voltage, Velocity<Velocity<Angle>>>> DRIVE_KA = VoltsPerRadianPerSecondSquared.of(0.009371);

    public static final Measure<Per<Voltage, Velocity<Angle>>> DRIVE_KP = VoltsPerRadianPerSecond.of(0.0043347);

    public static final double DRIVE_GEARING = 6.75;

    public static final Measure<Distance> DRIVE_CIRCUMFERENCE = Meters.of(0.05 * 2 * Math.PI); 
  
    // turn feedforward is only needed for sim
    public static final Measure<Per<Voltage, Velocity<Angle>>> TURN_KV = VoltsPerRadianPerSecond.of(0.37498);
    public static final Measure<Per<Voltage, Velocity<Velocity<Angle>>>> TURN_KA = VoltsPerRadianPerSecondSquared.of(0.00003572);

    public static final Measure<Velocity<Angle>> MAX_TURN_SPEED = RadiansPerSecond.of(32);

    public static final Measure<Per<Voltage, Angle>> TURN_KP = Volts.per(Radian).of(20);

    public static final double TURN_GEARING = 150/7;
  }
}
