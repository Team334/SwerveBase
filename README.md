# SwerveBase
A base project for future robots that has code for swerve drive and PhotonVision AprilTag processing.

## Features
- Abstract IO for modules / gyros allowing for easy switching between real life and sim.
- Device logging and data logging using SignalLogger, URCL, and Monologue.
- High frequency odometry thread for more accuracy.
- Automated pre-match self-check.
- Device configuration re-attempting.
- Device fault logging as telemetry for at-home testing.
- 254's Swerve Setpoint generator to prevent wheel slip.
- A custom class for PhotonVision AprilTag processing featuring filtering methods and standard deviation calculation.
- Wheel radius characterization (todo).
- Torque-current to voltage feedforward control for auton using Pathplanner/Choreo (todo).

## Inspiration
- [Team 1155 2024 Build Thread](https://www.chiefdelphi.com/t/frc-1155-the-sciborgs-2024-build-thread-open-alliance/441531)
- [Team 1155 Swerve Base](https://github.com/SciBorgs/Hydrogen/)
- [Team 3015 Code 2024](https://github.com/3015RangerRobotics/2024Public/tree/main/RobotCode2024/src/main/java/frc)
- [Team 353 Code 2024](https://github.com/POBots-353/2024RobotCode/tree/main)
- [CTRE Swerve API Odom Thread](https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/mechanisms/swerve/SwerveDrivetrain.OdometryThread.html)
- Team 6328 [2023](https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2023-build-thread/420691?page=2) and [2024](https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2024-build-thread/442736) Build Threads (specifically interested in vision)
- Team 6328 [2023](https://github.com/Mechanical-Advantage/RobotCode2023/tree/main) and [2024](https://github.com/Mechanical-Advantage/RobotCode2024) Robot Code (specifically interested in vision)
- [Team 5104 Vision SlideShow](https://docs.google.com/presentation/d/1ThMRapRsx5xbsswi_BTG8JsSiyXnA3HB3lH4m8eyqG4/edit?usp=sharing)


*(additional notes located in notes.md)*
