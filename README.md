# SwerveBase
A base project for future robots that has code for swerve drive, PhotonVision processing, and led strip control.

## Logging
For logging, the swerve base uses Monologue to log robot-specficic data. SignalLogger and URCL are used to log device-specific (motors/encoders) data.

## Faults
All faults are logged by SignalLogger/UCRL. During operation, faults may trigger subsystem alerts which will be displayed in elastic.

## Sources
- [Team 1155 Swerve Base](https://github.com/SciBorgs/Hydrogen/)
- [Team 3015 Base Code for 2024](https://github.com/3015RangerRobotics/2024Public/tree/main/RobotCode2024/src/main/java/frc)