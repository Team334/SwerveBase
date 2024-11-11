// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve.gyros;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static frc.lib.subsystem.SelfChecked.sequentialUntil;
import static frc.robot.Constants.SwerveConstants.ODOM_FREQUENCY;

import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.CTREUtil;
import frc.lib.FaultsTable.FaultType;

/** Pigeon Gyro Implementation. */
public class PigeonGyro implements GyroIO {
  private final Pigeon2 _pigeon;

  private final StatusSignal<Double> _yaw;

  private boolean _pigeonConfigError = false;
  
  public PigeonGyro(int pigeonId) {
    _pigeon = new Pigeon2(pigeonId);

    _yaw = _pigeon.getYaw();

    _pigeonConfigError |= CTREUtil.attempt(() -> _yaw.setUpdateFrequency(ODOM_FREQUENCY), _pigeon);
    _pigeonConfigError |= CTREUtil.attempt(() -> _pigeon.optimizeBusUtilization(), _pigeon);
  }

  @Override
  public Rotation2d getYaw() {
    // refreshed by odom thread
    return Rotation2d.fromDegrees(_yaw.getValueAsDouble());
  }

  @Override
  public boolean isConnected() {
    return true;
  }

  @Override
  public BaseStatusSignal getOdomSignal() {
    return _yaw;
  }

  @Override
  public Command selfCheck(BiConsumer<String, FaultType> faultAdder, BooleanSupplier hasError) {
    return sequentialUntil(
      hasError,
      runOnce(() -> {
        if (_pigeonConfigError) faultAdder.accept("Pigeon failed config, check total faults.", FaultType.ERROR);
        // TODO: the rest
      })
    );
  }
}
