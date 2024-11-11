// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve.gyros;

import com.ctre.phoenix6.BaseStatusSignal;

import edu.wpi.first.math.geometry.Rotation2d;

/** Dummy gyro that does nothing. */
public class NoGyro implements GyroIO {
  @Override
  public Rotation2d getYaw() {
    return new Rotation2d();
  }

  @Override
  public boolean isConnected() {
    return true;
  }

  @Override
  public BaseStatusSignal getOdomSignal() {
    return null;
  }
}
