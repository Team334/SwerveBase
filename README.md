# SwerveBase
A base project for future robots that has code for swerve drive, PhotonVision processing, and led strip control.

## Logging
For logging, the swerve base uses Monologue to log robot-specficic data. SignalLogger and URCL are used to log device-specific (motors/encoders) data.

## Alert System
Elastic alerts widgets are used for multiple purposes in this project. A FaultLogger class displays device faults (and failed device configurations) on one alerts widget as telemetry when testing. During prematch, there will be a prematch self-check command for each subsystem which will run test commands of the subsystem and alert the statuses of those commands. Checks for device faults/configuration errors are included in the self-check command.

## Odometery
For odometery, a custom thread running at a higher frequency (>50hz) in order to have more accurate odometery (according to ctre's document (here)[https://v6.docs.ctr-electronics.com/en/latest/docs/application-notes/update-frequency-impact.html]). There is also basic latency compensation on status signals using linear interpolation.

## Todo
- Finish vision template code (std devs calculation will require actual testing).
- Add sysid characterization for mechanisms.
- Add pathplanner (or choreo?) following/path generation.
- Add leds code.
- Add whatever is in notes md.

## Sources
- [Team 1155 Swerve Base](https://github.com/SciBorgs/Hydrogen/)
- [Team 3015 Code for 2024](https://github.com/3015RangerRobotics/2024Public/tree/main/RobotCode2024/src/main/java/frc)
- [Team 353 Code for 2024](https://github.com/POBots-353/2024RobotCode/tree/main)
- [CTRE Swerve API](https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/mechanisms/swerve/SwerveDrivetrain.html)