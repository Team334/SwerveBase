package frc.robot.subsystems.swerve;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Robot;

public class SimModule implements ModuleIO {
  private final DCMotorSim _driveMotor = new DCMotorSim(
    LinearSystemId.createDCMotorSystem(0, 0),
    DCMotor.getFalcon500(1),
    1 // 1:1 in sim
  );

  private final DCMotorSim _turnMotor = new DCMotorSim(
    LinearSystemId.createDCMotorSystem(0, 0),
    DCMotor.getFalcon500(1),
    1 // 1:1 in sim
  );


  // TODO: add pid controllers and proper units

  public SimModule() {}

  @Override
  public void setDriveTargets(double velocity, double acceleration) {
    _driveMotor.setInputVoltage(0);
    _driveMotor.update(Robot.kDefaultPeriod);
  }

  @Override
  public void setTurnAngle(double angle) {
    _turnMotor.setInputVoltage(0);
    _turnMotor.update(Robot.kDefaultPeriod);
  }

  @Override
  public double getDriveVelocity() {
    return _driveMotor.getAngularVelocityRPM();
  }

  @Override
  public double getTurnPosition() {
    return _turnMotor.getAngularPositionRotations();
  }
  
}
