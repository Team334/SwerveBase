// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.lib.subsystem.SelfChecked.sequentialUntil;
import static frc.robot.Constants.SwerveModuleConstants.*; // for neatness on can ids

import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Alert.AlertType;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Robot;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveModule.ControlMode;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.NavXGyro;
import frc.robot.subsystems.swerve.gyro.SimGyro;
import monologue.Logged;
import monologue.Annotations.Log;

public class Swerve extends AdvancedSubsystem implements Logged {
  private final SwerveModule _frontLeft;
  private final SwerveModule _frontRight;
  private final SwerveModule _backRight;
  private final SwerveModule _backLeft;

  private final List<SwerveModule> _modules;

  private final GyroIO _gyro;
  private Rotation2d _simYaw = new Rotation2d();

  private final SwerveDriveKinematics _kinematics = new SwerveDriveKinematics(SwerveConstants.MODULE_POSITIONS);

  private final SwerveDrivePoseEstimator _poseEstimator = new SwerveDrivePoseEstimator(
    _kinematics,
    getRawHeading(),
    getModulePositions(),
    new Pose2d(0, 0, getRawHeading())
  );

  /** The control of the drive motors in the swerve's modules. */
  @Log.NT(key = "Module Control Mode")
  public ControlMode controlMode = ControlMode.OPEN_LOOP;

  /** Whether to allow the modules in the drive to turn in place. */
  @Log.NT(key = "Allow Turn In Place")
  public boolean allowTurnInPlace = false;

  /** Whether the swerve is driven field oriented or not. */
  @Log.NT(key = "Field Oriented")
  public boolean isFieldOriented = false;

  /** Creates a new Swerve subsystem based on whether the robot is real or sim. */
  public static Swerve create() {
    if (RobotBase.isReal()) {
      return new Swerve(
        new RealModule(FRONT_LEFT_DRIVE_ID, FRONT_LEFT_TURN_ID, FRONT_LEFT_ENCODER_ID),
        new RealModule(FRONT_RIGHT_DRIVE_ID, FRONT_RIGHT_TURN_ID, FRONT_RIGHT_ENCODER_ID),
        new RealModule(BACK_RIGHT_DRIVE_ID, BACK_RIGHT_TURN_ID, BACK_RIGHT_ENCODER_ID),
        new RealModule(BACK_LEFT_DRIVE_ID, BACK_LEFT_TURN_ID, BACK_LEFT_ENCODER_ID),
        new NavXGyro()
      );
    } else {
      return new Swerve(
        new SimModule(),
        new SimModule(),
        new SimModule(), 
        new SimModule(),
        new SimGyro()
      );
    }
  }

  /** Creates a new Swerve. */
  public Swerve(
    ModuleIO frontLeft,
    ModuleIO frontRight,
    ModuleIO backRight,
    ModuleIO backLeft,
    GyroIO gyro
  ) {
    _frontLeft = new SwerveModule("Front Left Module", frontLeft);
    _frontRight = new SwerveModule("Front Right Module", frontRight);
    _backRight = new SwerveModule("Back Right Module", backRight);
    _backLeft = new SwerveModule("Back Left Module", backLeft);

    _gyro = gyro;

    _modules = List.of(_frontLeft, _frontRight, _backRight, _backLeft);
  }

  /** 
   * Drives the swerve drive.
   * 
   * @param velX The x velocity in meters per second. 
   * @param velY The y velocity in meters per second.
   * @param velOmega The rotational velocity in radians per second.
   */
  public void drive(double velX, double velY, double velOmega) {
    ChassisSpeeds chassisSpeeds;

    if (isFieldOriented) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        velX,
        velY,
        velOmega,
        getHeading()
      );
    } else {
      chassisSpeeds = new ChassisSpeeds(velX, velY, velOmega);
    }

    chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, Robot.kDefaultPeriod);

    log("Desired Chassis Speeds", chassisSpeeds);

    setModuleStates(_kinematics.toSwerveModuleStates(chassisSpeeds));
  }

  /** Returns the robot-relative ChassisSpeeds of the drive. */
  @Log.NT(key = "Chassis Speeds")
  public ChassisSpeeds getChassisSpeeds() {
    return _kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Get all the desired states of all the modules (in correct order). */
  @Log.NT(key = "Desired Module States")
  public SwerveModuleState[] getDesiredModuleStates() {
    return _modules.stream().map(SwerveModule::getDesiredState).toArray(SwerveModuleState[]::new);
  }
  
  /** Get all the module states (in correct order). */
  @Log.NT(key = "Module States")
  public SwerveModuleState[] getModuleStates() {
    return _modules.stream().map(SwerveModule::getModuleState).toArray(SwerveModuleState[]::new);
  }

  /** Get all the module positions (in correct order). */
  @Log.NT(key = "Module Positions")
  public SwerveModulePosition[] getModulePositions() {
    return _modules.stream().map(SwerveModule::getModulePosition).toArray(SwerveModulePosition[]::new);
  }

  /** Set all the module states (must be in correct order). */
  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_TRANSLATIONAL_SPEED.in(MetersPerSecond));

    for (int i = 0; i < _modules.size(); i++) {
      _modules.get(i).setModuleState(states[i], controlMode, allowTurnInPlace);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    for (SwerveModule module : _modules) {
      module.periodic();
    }
  }

  @Override
  public void simulationPeriodic() {
    _simYaw = _simYaw.plus(Rotation2d.fromRadians(getChassisSpeeds().omegaRadiansPerSecond * Robot.kDefaultPeriod));
  }

  /** Returns the pose of the drive from the pose estimator. */
  @Log.NT(key = "Pose")
  public Pose2d getPose() {
    return new Pose2d();
  }

  /** Returns the heading of the drive. */
  @Log.NT(key = "Heading")
  public Rotation2d getHeading() {
    // return getPose().getRotation();
    return getRawHeading();
  }

  /** Returns the raw heading of the gyro. */
  @Log.NT(key = "Raw Heading")
  public Rotation2d getRawHeading() {
    if (RobotBase.isReal()) {
      return _gyro.getYaw();
    } else {
      return _simYaw;
    }
  }

  @Override
  public Command selfCheck(BiConsumer<String, AlertType> alerter, BooleanSupplier hasError) {
    return sequentialUntil(
      hasError,
      _frontLeft.selfCheck(alerter, hasError),
      _frontRight.selfCheck(alerter, hasError),
      _backRight.selfCheck(alerter, hasError),
      _backLeft.selfCheck(alerter, hasError)
      // TODO: all other checks
    );
  }
}
