// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;

import edu.wpi.first.math.geometry.Rotation2d;

/** Dummy module that does nothing. */
public class NoModule implements ModuleIO {
  @Override
  public double getVelocity() {
    return 0;
  }

  @Override
  public void setVelocity(double velocity, boolean isOpenLoop) {}

  @Override
  public Rotation2d getAngle() {
    return new Rotation2d();
  }

  @Override
  public void setAngle(Rotation2d angle) {}

  @Override
  public double getPosition() {
    return 0;
  }
  
  @Override
  public BaseStatusSignal[] getOdomSignals() {
    return new BaseStatusSignal[]{};
  }

  @Override
  public void setDriveVoltage(double volts) {}

  @Override
  public void setTurnVoltage(double volts) {}
}
