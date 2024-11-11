// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.lib.subsystem.SelfChecked.sequentialUntil;
import static frc.robot.Constants.ModuleConstants.*; // for neatness on can ids
import static frc.robot.Constants.SwerveConstants.PIGEON_ID;
import static edu.wpi.first.wpilibj2.command.Commands.*; 

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import org.photonvision.simulation.VisionSystemSim;
import com.ctre.phoenix6.BaseStatusSignal;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.InputStream;
import frc.lib.FaultsTable.FaultType;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Robot;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.swerve.gyros.GyroIO;
import frc.robot.subsystems.swerve.gyros.NavXGyro;
import frc.robot.subsystems.swerve.gyros.NoGyro;
import frc.robot.subsystems.swerve.gyros.PigeonGyro;
import frc.robot.subsystems.swerve.modules.ModuleIO;
import frc.robot.subsystems.swerve.modules.NoModule;
import frc.robot.subsystems.swerve.modules.PerfectModule;
import frc.robot.subsystems.swerve.modules.RealModule;
import frc.robot.subsystems.swerve.modules.SimModule;
import frc.robot.util.SwerveSetpointGenerator;
import frc.robot.util.VisionPoseEstimator;
import frc.robot.util.SwerveSetpointGenerator.ModuleLimits;
import frc.robot.util.SwerveSetpointGenerator.SwerveSetpoint;
import frc.robot.util.VisionPoseEstimator.VisionPoseEstimate;
import monologue.Annotations.Log;

public class Swerve extends AdvancedSubsystem {
  private final SwerveModule _frontLeft;
  private final SwerveModule _frontRight;
  private final SwerveModule _backRight;
  private final SwerveModule _backLeft;

  private final List<SwerveModule> _modules;

  private final GyroIO _gyro;
  private Rotation2d _simYaw = new Rotation2d();

  private final SwerveDriveKinematics _kinematics = new SwerveDriveKinematics(SwerveConstants.MODULE_POSITIONS);

  private final SwerveDrivePoseEstimator _poseEstimator;
  private final SwerveDriveOdometry _simOdometry; // odometry to be used by sim vision

  // this is necessary for vision disambiguation due to vision latency
  private final TimeInterpolatableBuffer<Rotation2d> _headingBuffer = TimeInterpolatableBuffer.createBuffer(1.5); 

  private final OdometryThread _odomThread;
  private final ReentrantLock _odomUpdateLock = new ReentrantLock();

  // to log odom update successfulness (based on 353)
  private int _attemptedOdomUpdates = 0;
  private int _failedOdomUpdates = 0;

  // these values are updated by the odometry thread to prevent thread-safety issues
  // that can be caused when a status signal is being read while it is concurrently being updated (in the odom thread)
  private Rotation2d _cachedRawHeading;
  private SwerveModuleState[] _cachedModuleStates;
  private SwerveModulePosition[] _cachedModulePositions;
  private Pose2d _cachedEstimatedPose;
  private Pose2d _cachedSimOdomPose;

  private final List<VisionPoseEstimator> _cameras;
  private final VisionSystemSim _visionSim;
  private double _lastestVisionTimestamp = 0;

  private ChassisSpeeds _inputChassisSpeeds = new ChassisSpeeds();

  private final List<VisionPoseEstimate> _acceptedEstimates = new ArrayList<VisionPoseEstimate>(); // the accepted estimates (max 2) since the last cam retrieval
  private final List<VisionPoseEstimate> _rejectedEstimates = new ArrayList<VisionPoseEstimate>(); // the rejected estimates (max 2) since the last cam retrieval
  private final List<Pose3d> _detectedTargets = new ArrayList<>(); // the detected targets since the last cam retrieval

  // use the swerve setpoint generator instead of slew rate limiters for accel (this limits accel of each module individually)
  private SwerveSetpointGenerator _setpointGenerator = new SwerveSetpointGenerator(
    SwerveConstants.MODULE_POSITIONS,
    new ModuleLimits(
      SwerveConstants.MAX_TRANSLATIONAL_SPEED.in(MetersPerSecond),
      SwerveConstants.MAX_TRANSLATIONAL_ACCELERATION.in(MetersPerSecondPerSecond),
      ModuleConstants.MAX_TURN_SPEED.in(RadiansPerSecond)
    )
  );

  private SwerveSetpoint _prevSetpoint;

  // drive motor characterization
  private final SysIdRoutine _driveCharacterization;

  // turn motor characterization
  private final SysIdRoutine _turnCharacterization;

  @Log.NT(key = "Characterizing")
  private boolean _characterizing = false;

  /** Whether the velocity of the modules is controlled open-loop (FF only) or closed-loop (FF + PID). */
  @Log.NT(key = "Is Open Loop")
  public boolean isOpenLoop = true;

  /** Whether the swerve is driven field oriented or not. */
  @Log.NT(key = "Is Field Oriented")
  public boolean isFieldOriented = false;

  @Log.NT.Once(key = "Using Pigeon Gyro")
  private static boolean _usingPigeon = false;

  // select sim module type
  @Log.NT.Once(key = "Use Perfect Modules")
  private static boolean _usePerfectModules = false;
  
  // choose the desired simulated module type
  private static ModuleIO getSimModule() {
    if (_usePerfectModules) {
      return new PerfectModule();
    }

    return new SimModule();
  }

  /** Creates a new Swerve subsystem based on whether the robot is real or sim. */
  public static Swerve create() {
    if (RobotBase.isReal()) {
      return new Swerve(
        new RealModule(FRONT_LEFT_DRIVE_ID, FRONT_LEFT_TURN_ID, FRONT_LEFT_ENCODER_ID),
        new RealModule(FRONT_RIGHT_DRIVE_ID, FRONT_RIGHT_TURN_ID, FRONT_RIGHT_ENCODER_ID),
        new RealModule(BACK_RIGHT_DRIVE_ID, BACK_RIGHT_TURN_ID, BACK_RIGHT_ENCODER_ID),
        new RealModule(BACK_LEFT_DRIVE_ID, BACK_LEFT_TURN_ID, BACK_LEFT_ENCODER_ID),
        _usingPigeon ? new PigeonGyro(PIGEON_ID) : new NavXGyro()
      );
    } else {
      return new Swerve(
        getSimModule(),
        getSimModule(),
        getSimModule(),
        getSimModule(),
        new NoGyro()
      );
    }
  }

  /**
   * Creates a dummy Swerve subsystem with dummy modules that don't do anything.
   * (use this instead of commenting out all swerve code when swerve isn't on bot)
   */
  public static Swerve none() {
    return new Swerve(
      new NoModule(),
      new NoModule(),
      new NoModule(),
      new NoModule(),
      new NoGyro()
    );
  }

  /** 
   * An odometry thread that updates module status signals at a specified frequency and feeds the signals into a pose estimator.
   */
  public class OdometryThread {
    private Notifier _notifier = new Notifier(this::update);
    private final double _frequency;

    private BaseStatusSignal[] _signals = new BaseStatusSignal[0];

    // using dummy modules or in sim
    private boolean _noSignals = false;

    public OdometryThread(double frequency) {
      _frequency = frequency;
      _notifier.setName("Odometry Thread");

      // implies that dummy module is being used or sim module is being used
      _noSignals = _modules.get(0).getOdomSignals().length == 0;

      if (_noSignals) return;

      _signals = new BaseStatusSignal[3 * 4 + (_usingPigeon ? 1 : 0)]; // 3 signals per module + possible pigeon yaw signal

      for (int i = 0; i < _modules.size(); i++) {
        BaseStatusSignal[] moduleSignals = _modules.get(i).getOdomSignals();
        
        _signals[(i*3) + 0] = moduleSignals[0];
        _signals[(i*3) + 1] = moduleSignals[1];
        _signals[(i*3) + 2] = moduleSignals[2];
      }

      if (_usingPigeon) _signals[_signals.length - 1] = _gyro.getOdomSignal();

      // signal frequency set inside modules / gyro
    }

    /** Refreshes all the odom status signals, returning true if the action failed. */
    private boolean refreshStatusSignals() {
      if (!_noSignals) {
        return BaseStatusSignal.refreshAll(_signals).isError();
      }

      return false;
    }

    /** Starts the odom thread. */
    public void start() {
      _notifier.startPeriodic(1 / _frequency);
    }

    /** Stops the odom thread. */
    public void stop() {
      _notifier.stop();
    }

    private void update() {
      _odomUpdateLock.lock();
      
      _attemptedOdomUpdates ++;

      boolean willOdomUpdateFail = false;

      // refresh status signals, and check if successful refresh
      willOdomUpdateFail = refreshStatusSignals();

      // in the case of a navx, a different refresh and success check is used
      if (_gyro instanceof NavXGyro) {
        willOdomUpdateFail |= !_gyro.isConnected();
      }

      // updated all cached device data
      // if a refresh failed, then the cached data would just
      // be from the last successful refresh
      updateCached();
      
      // log everything now instead of the main thread to 
      // reduce logging latency
      log("Module States", getModuleStates());
      log("Desired Module States", getDesiredModuleStates());
      log("Input Chassis Speeds", _inputChassisSpeeds);
      log("Desired Chassis Speeds", _prevSetpoint.chassisSpeeds());
      log("Chassis Speeds", getChassisSpeeds());
      log("Module Positions", getModulePositions());
      log("Raw Heading", getRawHeading());

      // all the devices didn't successfully refresh, so don't update odom
      if (willOdomUpdateFail) {
        _failedOdomUpdates ++;
        _odomUpdateLock.unlock();
        return;
      }

      // all devices refreshed, so update odom with new device data
      _cachedEstimatedPose = _poseEstimator.update(getRawHeading(), getModulePositions());
      _cachedSimOdomPose = _simOdometry.update(getRawHeading(), getModulePositions());

      _headingBuffer.addSample(Timer.getFPGATimestamp(), getHeading());

      log("Robot Pose", getPose()); // log the pose at a higher frequency (also with less latency)
      log("Robot Heading", getHeading());

      _odomUpdateLock.unlock();
    }
  }

  /** Creates a new Swerve. */
  public Swerve(
    ModuleIO frontLeft,
    ModuleIO frontRight,
    ModuleIO backRight,
    ModuleIO backLeft,
    GyroIO gyro
  ) {
    _frontLeft = new SwerveModule("Front Left Module", frontLeft);
    _frontRight = new SwerveModule("Front Right Module", frontRight);
    _backRight = new SwerveModule("Back Right Module", backRight);
    _backLeft = new SwerveModule("Back Left Module", backLeft);

    _gyro = gyro;
    _modules = List.of(_frontLeft, _frontRight, _backRight, _backLeft);

    _cameras = VisionPoseEstimator.buildFromCameras(VisionConstants.CAM_CONSTANTS);

    updateCached();

    _poseEstimator = new SwerveDrivePoseEstimator(
      _kinematics,
      getRawHeading(),
      getModulePositions(),
      new Pose2d(0, 0, getRawHeading())
    );

    _simOdometry = new SwerveDriveOdometry(
      _kinematics,
      getRawHeading(),
      getModulePositions(),
      new Pose2d(0, 0, getRawHeading())
    );

    _cachedEstimatedPose = _poseEstimator.getEstimatedPosition();
    _cachedSimOdomPose = _simOdometry.getPoseMeters();
    

    // motor logging handled by signal logger
    _driveCharacterization = new SysIdRoutine(
      new SysIdRoutine.Config(Volts.per(Seconds).of(1.2), Volts.of(7), Seconds.of(4)),
      new SysIdRoutine.Mechanism(volts -> _modules.forEach(m -> m.setDriveVoltage(volts.in(Volts))), null, this)
    );

    // motor logging handled by signal logger
    _turnCharacterization = new SysIdRoutine(
      new SysIdRoutine.Config(Volts.per(Seconds).of(1), Volts.of(7), Seconds.of(4)),
      new SysIdRoutine.Mechanism(volts -> _modules.forEach(m -> m.setTurnVoltage(volts.in(Volts))), null, this)
    );

    displayRoutines();

    _odomThread = new OdometryThread(SwerveConstants.ODOM_FREQUENCY);

    if (RobotBase.isSimulation()) {
      _visionSim = new VisionSystemSim("main");
      _visionSim.addAprilTags(FieldConstants.FIELD_LAYOUT);

      _cameras.forEach(cam -> _visionSim.addCamera(cam.getSimCamera(), cam.robotToCam));
    } else {
      _visionSim = null;

      _odomThread.start(); // use threading on rio only
    }

    resetSetpointGenerator();
  }

  // wraps around drive sysid routines
  private Command buildRoutine(Command routine) {
    return sequence(
      runOnce(() -> _characterizing = true),
      run(() -> _modules.forEach(m -> m.setDriveVoltage(0))).withTimeout(2),
      routine,
      runOnce(() -> _characterizing = false)
    ).withName("SysId Routine");
  }

  // display the sysid routines on smart dashboard
  private void displayRoutines() {
    SmartDashboard.putData("Swerve Drive Quasi-Static Forward", buildRoutine(
      _driveCharacterization.quasistatic(Direction.kForward)
    ));
    SmartDashboard.putData("Swerve Drive Quasi-Static Reverse", buildRoutine(
      _driveCharacterization.quasistatic(Direction.kReverse)
    ));
    SmartDashboard.putData("Swerve Drive Dynamic Forward", buildRoutine(
      _driveCharacterization.dynamic(Direction.kForward)
    ));
    SmartDashboard.putData("Swerve Drive Dynamic Reverse", buildRoutine(
      _driveCharacterization.dynamic(Direction.kReverse)
    ));

    SmartDashboard.putData("Swerve Turn Quasi-Static Forward", buildRoutine(
      _turnCharacterization.quasistatic(Direction.kForward)
    ));
    SmartDashboard.putData("Swerve Turn Quasi-Static Reverse", buildRoutine(
      _turnCharacterization.quasistatic(Direction.kReverse)
    ));
    SmartDashboard.putData("Swerve Turn Dynamic Forward", buildRoutine(
      _turnCharacterization.dynamic(Direction.kForward)
    ));
    SmartDashboard.putData("Swerve Turn Dynamic Reverse", buildRoutine(
      _turnCharacterization.dynamic(Direction.kReverse)
    ));

    SmartDashboard.putData("Swerve Drive Motors Verify", buildRoutine(
      run(() -> _modules.forEach(m -> m.setDriveVoltage(3))).withTimeout(5)
    ));

    SmartDashboard.putData("Swerve Turn Motors Verify", buildRoutine(
      run(() -> _modules.forEach(m -> m.setTurnVoltage(3))).withTimeout(5)
    ));
  }

  /**
   * Creates a new Command that drives the drive. The driving configuration is set with the
   * {@link #isFieldOriented}, {@link #isOpenLoop}, and {@link #allowTurnInPlace} members.
   * 
   * @param velX The x velocity in meters per second.
   * @param velY The y velocity in meters per second.
   * @param velOmega The rotational velocity in radians per second.
   */
  public Command drive(InputStream velX, InputStream velY, InputStream velOmega) {
    return run(() -> {
      drive(
        velX.get(),
        velY.get(),
        velOmega.get()
      );
    }).beforeStarting(() -> {
      // reset the accel limiters since the command changed velocity
      resetSetpointGenerator();
    }).withName("Drive");
  }

  /**
   * "Brakes" the swerve drive by angling all the modules to form an "X", making it so that the bot
   * can't move anywhere.
   */
  public Command brake() {
    return run(() -> {
      setModuleStates(new SwerveModuleState[] {
        new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45))
      });
    }).beforeStarting(() -> {
      _inputChassisSpeeds = new ChassisSpeeds();
    }).withName("Brake");
  }

  /** Toggles {@link #isFieldOriented} by braking the swerve and changing the drive orientation. */
  public Command toggleOriented() {
    return brake().withTimeout(0.5)
                  .andThen(() -> isFieldOriented = !isFieldOriented)
                  .withName("Toggle Orient");
  }

  /** 
   * Drives the swerve drive. The driving configuration is set with the 
   * {@link #isFieldOriented} and {@link #isOpenLoop} members.
   * 
   * @param velX The x velocity in meters per second. 
   * @param velY The y velocity in meters per second.
   * @param velOmega The rotational velocity in radians per second.
   */
  public void drive(double velX, double velY, double velOmega) {
    // input speeds as a chassis speeds object
    _inputChassisSpeeds = new ChassisSpeeds(
      velX,
      velY,
      velOmega
    );

    // final desired speeds
    ChassisSpeeds desiredChassisSpeeds = _inputChassisSpeeds;

    if (isFieldOriented) {
      // turn desired speeds into robot-relative if needed
      desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        desiredChassisSpeeds,
        getHeading()
      );
    }

    // desired module states
    SwerveModuleState[] moduleStates;

    // modify desired chassis speeds to make them achievable based on the module max drive speed
    moduleStates = _kinematics.toSwerveModuleStates(desiredChassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.MAX_TRANSLATIONAL_SPEED);
    desiredChassisSpeeds = _kinematics.toChassisSpeeds(moduleStates);

    // use the setpoint generator to limit accel on the new setpoint
    _prevSetpoint = _setpointGenerator.generateSetpoint(_prevSetpoint, desiredChassisSpeeds);

    setModuleStates(_prevSetpoint.moduleStates());
  }
  
  private void resetSetpointGenerator() {
    _prevSetpoint = new SwerveSetpoint(
      getChassisSpeeds(),
      getModuleStates(),
      new double[4]
    );
  }

  // updates all cached data (should only be called in odom thread)
  private void updateCached() {
    updateCachedRawHeading();
    updateCachedModuleStates();
    updateCachedModulePositions();
  }

  // updates the cached raw heading (should only be called in odom thread)
  private void updateCachedRawHeading() {
    if (RobotBase.isReal()) {
      _cachedRawHeading = _gyro.getYaw();
    } else {
      _cachedRawHeading = _simYaw;
    }
  }

  // updates the cached module states (should only be called in odom thread)
  private void updateCachedModuleStates() {
    _cachedModuleStates = _modules.stream().map(SwerveModule::getModuleState).toArray(SwerveModuleState[]::new);
  }

  // updates the cached module states (should only be called in odom thread)
  private void updateCachedModulePositions() {
    _cachedModulePositions = _modules.stream().map(SwerveModule::getModulePosition).toArray(SwerveModulePosition[]::new);
  }

  /** Returns the robot-relative ChassisSpeeds of the drive. */
  public ChassisSpeeds getChassisSpeeds() {
    return _kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Get all the desired module states (in correct order). */
  public SwerveModuleState[] getDesiredModuleStates() {
    return _modules.stream().map(SwerveModule::getDesiredState).toArray(SwerveModuleState[]::new);
  }
  
  /** Get all the module states since the last odom update (in correct order). */
  public SwerveModuleState[] getModuleStates() {
    return _cachedModuleStates;
  }

  /** 
   * Get all the modules positions since the last odom update (in correct order).
   */
  public SwerveModulePosition[] getModulePositions() {
    return _cachedModulePositions;
  }

  /** Set all the module states (must be in correct order). */
  public void setModuleStates(SwerveModuleState[] states) {
    for (int i = 0; i < _modules.size(); i++) {
      _modules.get(i).setModuleState(states[i], isOpenLoop);
    }
  }

  // updates the pose estimator with potential vision estimates
  private void updateVisionPoseEstimates() {
    _acceptedEstimates.clear();
    _rejectedEstimates.clear();
    _detectedTargets.clear();

    _cameras.forEach(cam -> {
      var possibleEstimate = cam.getEstimatedPose(_lastestVisionTimestamp, this::getHeadingAtTime);
      cam.logLatestEstimate();

      if (possibleEstimate.isEmpty()) return;
      
      VisionPoseEstimate estimatedPose = possibleEstimate.get();
      
      // add all detected ids into detected targets list
      for (int tagId : estimatedPose.detectedTags()) _detectedTargets.add(FieldConstants.FIELD_LAYOUT.getTagPose(tagId).get());

      // then check if the pose is valid for the estimator
      if (!estimatedPose.isValid()) { 
        _rejectedEstimates.add(estimatedPose); 
        return;
      }
      
      _acceptedEstimates.add(estimatedPose); // if valid, add to accepted
    });

    // first sort by timestamp, an earlier timestamp must come first so it can change
    // all the poses in the buffer in front of it, and that way the later timestamp will be able
    // to further correct the changed poses, also if two estimates fall in the same timestamp,
    // they must be sorted by increasing std devs so that the final pose in the timestamp will closer match
    // the less noisy vision estimate (with lower std devs) see this:
    // https://github.com/wpilibsuite/allwpilib/pull/4917#issuecomment-1376178648
    _acceptedEstimates.sort(VisionPoseEstimate.comparator);
    
    log("Accepted Estimates", _acceptedEstimates.stream().map(VisionPoseEstimate::pose).toArray(Pose3d[]::new));
    log("Rejected Estimates", _rejectedEstimates.stream().map(VisionPoseEstimate::pose).toArray(Pose3d[]::new));

    log("Detected Targets", _detectedTargets.toArray(Pose3d[]::new));

    if (_acceptedEstimates.size() == 0) return;
    _lastestVisionTimestamp = _acceptedEstimates.get(_acceptedEstimates.size() - 1).timestamp();

    if (RobotBase.isSimulation()) return; // idk about this

    _odomUpdateLock.lock();
    _acceptedEstimates.forEach(e -> _poseEstimator.addVisionMeasurement(e.pose().toPose2d(), e.timestamp(), e.stdDevs()));
    _odomUpdateLock.unlock();
  }

  @Override
  public void periodic() {
    super.periodic();

    if (!_characterizing) {
      for (SwerveModule module : _modules) {
        module.periodic();
      }
    }

    updateVisionPoseEstimates();

    double odomUpdateSuccessPercentage = -1;

    if (_attemptedOdomUpdates != 0) {
      odomUpdateSuccessPercentage = (double) (_attemptedOdomUpdates - _failedOdomUpdates) / _attemptedOdomUpdates * 100.0;
    }

    log("Odometry Update Success %", odomUpdateSuccessPercentage);
  }

  @Override
  public void simulationPeriodic() {
    _simYaw = _simYaw.plus(Rotation2d.fromRadians(getChassisSpeeds().omegaRadiansPerSecond * Robot.kDefaultPeriod));

    _odomThread.update(); // note that locks won't have any effect since this is synchronized
    log("Sim Odom Pose", _cachedSimOdomPose);

    _visionSim.update(_cachedSimOdomPose);
  }

  /** Returns the pose of the drive from the pose estimator. */
  public Pose2d getPose() {
    return _cachedEstimatedPose;
  }

  /** Resets the pose of the pose estimator. */
  public void resetPose(Pose2d newPose) {
    _odomUpdateLock.lock();
    
    _poseEstimator.resetPosition(
      getRawHeading(),
      getModulePositions(),
      newPose
    );
    
    _simOdometry.resetPosition(
      getRawHeading(),
      getModulePositions(),
      newPose
    );

    _odomUpdateLock.unlock();
  }

  /** Resets the heading of the pose estimator. */
  public void resetHeading(Rotation2d newHeading) {
    Pose2d oldPose = getPose();
    Pose2d newPose = new Pose2d(oldPose.getTranslation(), newHeading);

    resetPose(newPose);
  }

  /** Returns the heading of the drive. */
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /**
   * Gets the heading at the given timestamp. If the timestamp is less than (currentTime - 1.5s), 
   * the current heading is returned (same happens if the FPGA time is less than 1.5 seconds).
   */
  public Rotation2d getHeadingAtTime(double timestamp) {
    var possibleHeading = _headingBuffer.getSample(timestamp);

    if (possibleHeading.isPresent()) return possibleHeading.get();
    
    return getHeading();
  }

  /** 
   * Returns the cached raw heading since the last odom update.
   */
  public Rotation2d getRawHeading() {
    return _cachedRawHeading;
  }

  @Override
  public Command selfCheck(BiConsumer<String, FaultType> faultAdder, BooleanSupplier hasError) {
    return sequentialUntil(
      hasError,
      _frontLeft.selfCheck(faultAdder, hasError),
      _frontRight.selfCheck(faultAdder, hasError),
      _backRight.selfCheck(faultAdder, hasError),
      _backLeft.selfCheck(faultAdder, hasError)
      // TODO: all other checks
    );
  }
}
