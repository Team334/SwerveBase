// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import static frc.robot.util.Misc.sequentialUntil;

import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Alert.AlertType;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants.SwerveConstants;
import monologue.Logged;
import monologue.Annotations.Log;

public class Swerve extends AdvancedSubsystem implements Logged {
  private final SwerveModule _frontLeft;
  private final SwerveModule _frontRight;
  private final SwerveModule _backRight;
  private final SwerveModule _backLeft;

  private final List<SwerveModule> _modules;

  /** Creates a new Swerve subsystem based on whether the robot is real or sim. */
  public static Swerve create() {
    if (RobotBase.isReal()) {
      return new Swerve(
        new RealModule(0, 0, 0),
        new RealModule(0, 0, 0),
        new RealModule( 0, 0, 0),
        new RealModule(0, 0, 0)
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

  /** Get all the desired states of all the modules (in correct order). */
  @Log.NT
  public SwerveModuleState[] getDesiredModuleStates() {
    return _modules.stream().map(SwerveModule::getDesiredState).toArray(SwerveModuleState[]::new);
  }
  
  /** Get all the module states (in correct order). */
  @Log.NT
  public SwerveModuleState[] getModuleStates() {
    return _modules.stream().map(SwerveModule::getState).toArray(SwerveModuleState[]::new);
  }

  /** Set all the module states (must be in correct order). */
  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_SPEED.magnitude());

    for (int i = 0; i < _modules.size(); i++) {
      _modules.get(i).setState(states[i]);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
