# SwerveBase
A base project for future robots that has code for swerve drive, PhotonVision processing, and led strip control.

## Logging
For logging, the swerve base uses Monologue to log robot-specficic data. SignalLogger and URCL are used to log device-specific (motors/encoders) data.

## Alert System
Elastic alerts widgets are used for multiple purposes in this project. A FaultLogger class displays device faults (and failed device configuration) on one alerts widget as telemetry when testing. During prematch, there will be a prematch self-check command for each subsystem which will run test commands of the subsystem and alert the statuses of those commands. Checks for device faults are included in the self-check command.

## Sources
- [Team 1155 Swerve Base](https://github.com/SciBorgs/Hydrogen/)
- [Team 3015 Code for 2024](https://github.com/3015RangerRobotics/2024Public/tree/main/RobotCode2024/src/main/java/frc)
- [Team 353 Code for 2024](https://github.com/POBots-353/2024RobotCode/tree/main)
- [CTRE Swerve API](https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/mechanisms/swerve/SwerveDrivetrain.html)