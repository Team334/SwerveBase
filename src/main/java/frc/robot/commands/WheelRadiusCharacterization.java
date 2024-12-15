// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.FaultLogger;
import frc.lib.FaultsTable.FaultType;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.Swerve;

// TODO
/**
 * Finds the radius of the drive wheels.
 */
public class WheelRadiusCharacterization extends Command {
  private final Swerve _swerve;
  private final double _driveRadius = Math.hypot(SwerveConstants.DRIVE_LENGTH.magnitude() / 2, SwerveConstants.DRIVE_WIDTH.magnitude() / 2); 

  private double _lastestGyroReading;
  private double _accumulatedGyroReading;

  private double _effectiveWheelRadius;

  private double[] _startWheelPositions;
  
  /** Creates a new WheelRadiusCharacterization. */
  public WheelRadiusCharacterization(Swerve swerve) {
    _swerve = swerve;
    addRequirements(_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _lastestGyroReading = _swerve.getRawHeading().getRadians();
    _accumulatedGyroReading = 0;

    _startWheelPositions = _swerve.getWheelRadiusCharacterizationPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _swerve.drive(0, 0, 1);

    _accumulatedGyroReading += MathUtil.angleModulus(_swerve.getRawHeading().getRadians() - _lastestGyroReading);
    _lastestGyroReading = _swerve.getRawHeading().getRadians();

    double averageWheelPosition = 0;
    double[] wheelPositions = _swerve.getWheelRadiusCharacterizationPosition();
    
    for(int i = 0; i < wheelPositions.length; i++){
      averageWheelPosition += Math.abs(wheelPositions[i] - _startWheelPositions[i]);
    }

    averageWheelPosition /= wheelPositions.length;

    _effectiveWheelRadius = (_accumulatedGyroReading * _driveRadius) / averageWheelPosition;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _swerve.drive(0, 0, 0);

    if (_accumulatedGyroReading <= Math.PI * 2) {
      FaultLogger.report("Need more information for characterization", FaultType.ERROR);
    }
    else{
      System.out.println("Wheel Radius (inches): " + Units.metersToInches(_effectiveWheelRadius));
    }
  }
}
