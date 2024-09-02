# SwerveBase
A base project for future robots that has code for swerve drive, PhotonVision processing, and led strip control.

## Project Structure
The project is Command-Based, and structured to support easy switching between real-life testing and sim testing. Every subsystem has some sort of hardware "IO" (input-output) it controls, which can be some physical io (like an arm motor) or some simulated io (DCMotorSim) depending on whether the code is being run in sim mode or not. Because of this every subsystem will have a private `_io` member which is an implementation of an interface describing an io for that subsystem. For example, an arm io interface may have methods `setPosition` and `getPosition`, which are implemented differently by a "real io" and a "sim io", either of which can be used as `_io` in an "arm subsystem", the use case depending on whether the code is being simulated or not. (INSPIRED BY TEAM 1155 SEE BELOW)

## Logging
For logging, the swerve base uses Monologue to log robot-specific data. SignalLogger and URCL are used to log device-specific (motors/encoders) data.

## Automated Prematch Check
During prematch, there will be a prematch self-check command for each subsystem which will run test commands of the subsystem and alert the statuses of those commands. Checks for device faults/configuration errors are also included in the self-check command. Meanwhile during regular testing, a FaultLogger class (team 1155 library) displays device faults (and failed device configurations) on a different alerts widget as telemetry (since SignalLogger doesn't have telemetry). (INSPIRED BY 353 AND 3015 SEE BELOW)

## Odometry
For odometry, a custom thread running at a higher frequency (>50hz) is used in order to have more accurate odometry, since this allows a more frequent integration of the robot's twist movements which better defines the full movement of the robot and ultimately its pose (more about this [here](https://v6.docs.ctr-electronics.com/en/latest/docs/application-notes/update-frequency-impact.html)). In order to prevent concurrency issues, the device data (status signals) from the modules and gyro is refreshed in the odom thread and cached to be used whenever in the main thread. The device data however is logged immediately after it's refreshed in the odom thread to prevent the data from being logged some time later after it was actually received (latency pretty much).

## Full Pose Estimation
Pose estimation is a combination of odometry and vision measurements. WPILib's `SwerveDrivePoseEstimator` holds a buffer (of limited history size) of estimated poses each with a corresponding timestamp. Every time odom is updated, the robot's new pose is added into this buffer at the timestamp it was made. When a vision measurement of the robot's pose is made, the robot's pose at the timestamp of the vision measurement is fetched from the buffer and is moved towards the vision measurement by a certain amount (this depends on the vision measurement standard deviations at that moment as well as the configured odometry standard deviations). The twists between each pose ahead of that timestamp in the buffer are added to this new pose estimate (like a "replay") to get the robot's new estimated pose at the current timestamp. The vision standard deviations for the given vision measurement define the noise of the pose measurement. Corner noise increases rapidly as distance increases (due to target size decreasing rapidly with distance), so std devs are "stddevs = distance^2". A larger amount of noisy corners will result in a less noisy pose as compared to a smaller amount of noisy corners, which is why pose noise is smaller with multiple tags as compared to a single tag, making the equation "stddevs = (baseSingleTag or baseMultiTag) * (distance^2)". In a multi-camera setup, one camera might have a higher fov than another (fov also affects target size in the image) or a different resolution (lower resolution increases corner noise) than another, which is why some "cameraStdDevsFactor" is needed, so the final equation is "stddevs = (baseSingleTag or baseMultiTag) * (distance^2) * cameraStdDevsFactor". (NOTE: This is just a heuristic, and there can be other factors like whether the robot is moving or not which affects vision noise. Since this is a heuristic it isn't 100% accurate, and its accuracy should be compared with actual std devs calculated by advantage scope for example) (EQUATION INSPIRED BY TEAM 5104 AND TEAM 6328 SEE BELOW)

## More on Vision
(write stuff here TODO)

## Todo
(this is def not everything)
### Swerve Drive
- Switch to built-in encoder on swerve drive modules, which is updated by abs encoder (this is needed to achieve 1kHz pid loop).
- 6328 Wheel radius characterization.
### Vision
- Actually test out heuristic.
- Tune heuristic.
- How to calibrate with mrcal/mrgringham by recording a video (check out team 1155 in their 2024 build thread).
- Best way to do multi-camera setup/potential "camera coverage" calculator using given cam positions and fovs.
### Logging/Alerts
- Stay tuned with all the wpilib updates (I think 6328 Alerts are a part of wpilib now).
### Everything else
- Figure out how to use sysid, set it up for all control loops.
- Research on whether to stick with pathplanner or switch to choreo.
- Research on different code structures (like using finite state machines).
- Nice led code (based on some sort of "robot state").


## Inspiration
- [Team 1155 2024 Build Thread](https://www.chiefdelphi.com/t/frc-1155-the-sciborgs-2024-build-thread-open-alliance/441531)
- [Team 1155 Swerve Base](https://github.com/SciBorgs/Hydrogen/)
- [Team 3015 Code 2024](https://github.com/3015RangerRobotics/2024Public/tree/main/RobotCode2024/src/main/java/frc)
- [Team 353 Code 2024](https://github.com/POBots-353/2024RobotCode/tree/main)
- [CTRE Swerve API Odom Thread](https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/mechanisms/swerve/SwerveDrivetrain.OdometryThread.html)
- Team 6328 [2023](https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2023-build-thread/420691?page=2) and [2024](https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2024-build-thread/442736) Build Threads (specifically interested in vision)
- Team 6328 [2023](https://github.com/Mechanical-Advantage/RobotCode2023/tree/main) and [2024](https://github.com/Mechanical-Advantage/RobotCode2024) Robot Code (specifically interested in vision)
- [Team 5104 Vision SlideShow](https://docs.google.com/presentation/d/1ThMRapRsx5xbsswi_BTG8JsSiyXnA3HB3lH4m8eyqG4/edit?usp=sharing)
