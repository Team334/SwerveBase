// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import static frc.robot.util.Misc.sequentialUntil;
import static frc.robot.Constants.SwerveModuleConstants.*; // for neatness on can ids

import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Alert.AlertType;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveModule.ControlMode;
import monologue.Logged;
import monologue.Annotations.Log;

public class Swerve extends AdvancedSubsystem implements Logged {
  private final SwerveModule _frontLeft;
  private final SwerveModule _frontRight;
  private final SwerveModule _backRight;
  private final SwerveModule _backLeft;

  private final List<SwerveModule> _modules;

  private SwerveDriveKinematics _kinematics = new SwerveDriveKinematics(SwerveConstants.WHEEL_LOCATIONS);

  /** The control of the drive motors in the swerve's modules. */
  public ControlMode controlMode = ControlMode.OPEN_LOOP;

  /** Whether to allow the modules in the drive to turn in place. */
  public boolean allowTurnInPlace = false;

  /** Creates a new Swerve subsystem based on whether the robot is real or sim. */
  public static Swerve create() {
    if (RobotBase.isReal()) {
      return new Swerve(
        new RealModule(FRONT_LEFT_DRIVE_ID, FRONT_LEFT_TURN_ID, FRONT_LEFT_ENCODER_ID),
        new RealModule(FRONT_RIGHT_DRIVE_ID, FRONT_RIGHT_TURN_ID, FRONT_RIGHT_ENCODER_ID),
        new RealModule(BACK_RIGHT_DRIVE_ID, BACK_RIGHT_TURN_ID, BACK_RIGHT_ENCODER_ID),
        new RealModule(BACK_LEFT_DRIVE_ID, BACK_LEFT_TURN_ID, BACK_LEFT_ENCODER_ID)
      );
    } else {
      return new Swerve(
        new SimModule(),
        new SimModule(),
        new SimModule(), 
        new SimModule()
      );
    }
  }

  /** Creates a new Swerve. */
  public Swerve(
    ModuleIO frontLeft,
    ModuleIO frontRight,
    ModuleIO backRight,
    ModuleIO backLeft
  ) {
    _frontLeft = new SwerveModule("Front Left Module", frontLeft);
    _frontRight = new SwerveModule("Front Right Module", frontRight);
    _backRight = new SwerveModule("Back Right Module", backRight);
    _backLeft = new SwerveModule("Back Left Module", backLeft);

    _modules = List.of(_frontLeft, _frontRight, _backRight, _backLeft);
  }

  /** 
   * Drives the swerve drive.
   * 
   * @param chassisSpeeds The chassis speeds to drive the swerve at.
   * @param isFieldOriented Whether the speeds are robot relative or field oriented.
   */
  public void drive(ChassisSpeeds chassisSpeeds, boolean isFieldOriented) {
    
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

  /** Set all the module states (must be in correct order). */
  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_SPEED.magnitude());

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
