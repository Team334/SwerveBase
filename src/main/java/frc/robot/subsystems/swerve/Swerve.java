// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.lib.subsystem.SelfChecked.sequentialUntil;
import static frc.robot.Constants.SwerveModuleConstants.*; // for neatness on can ids

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Alert.AlertType;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Robot;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.swerve.SwerveModule.ControlMode;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.NavXGyro;
import frc.robot.subsystems.swerve.gyro.SimGyro;
import frc.robot.util.VisionPoseEstimator;
import frc.robot.util.VisionPoseEstimator.VisionPoseEstimate;
import monologue.Logged;
import monologue.Annotations.Log;

public class Swerve extends AdvancedSubsystem implements Logged {
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

  private final OdometryThread _odomThread;
  private final ReentrantLock _odomUpdateLock = new ReentrantLock();

  // to log odom update successfulness (based on 353)
  private int _attemptedOdomUpdates = 0;
  private int _failedOdomUpdates = 0;

  private ChassisSpeeds _desiredChassisSpeeds = new ChassisSpeeds();

  // these values are updated by the odometry thread to prevent thread-safety issues
  // that can be caused when a status signal is being read while it is concurrently being updated (in the odom thread)
  private Rotation2d _cachedRawHeading;
  private SwerveModuleState[] _cachedModuleStates;
  private SwerveModulePosition[] _cachedModulePositions;
  private Pose2d _cachedPose;
  private Pose2d _cachedSimOdomPose;

  private final List<VisionPoseEstimator> _cameras;
  private final VisionSystemSim _visionSim;
  private double _lastestVisionTimestamp = 0;

  private final List<VisionPoseEstimate> _acceptedEstimates = new ArrayList<VisionPoseEstimate>(); // the accepted estimates (max 2) since the last cam retrieval
  private final List<VisionPoseEstimate> _rejectedEstimates = new ArrayList<VisionPoseEstimate>(); // the rejected estimates (max 2) since the last cam retrieval
  private final List<Pose3d> _detectedTargets = new ArrayList<>(); // the detected targets since the last cam retrieval

  // for demo usage only, shows how faster odom depicts the robot's movement better than slower (possibly set this up later)
  // private final Deque<Pose2d> _fasterOdomPoses = new ArrayDeque<>();
  // private final Deque<Pose2d> _slowerOdomPoses = new ArrayDeque<>();
  // private final Pose2d _fasterOdomLastPose = new Pose2d();
  // private final Pose2d _slowerOdomLastPose = new Pose2d();

  /** The control of the drive motors in the swerve's modules. */
  @Log.NT(key = "Module Control Mode")
  public ControlMode controlMode = ControlMode.OPEN_LOOP;

  /** Whether to allow the modules in the drive to turn in place. */
  @Log.NT(key = "Allow Turn In Place")
  public boolean allowTurnInPlace = false;

  /** Whether the swerve is driven field oriented or not. */
  @Log.NT(key = "Drive Orientation")
  public DriveOrientation driveOrientation = DriveOrientation.ROBOT_ORIENTED;

  /** Creates a new Swerve subsystem based on whether the robot is real or sim. */
  public static Swerve create() {
    if (RobotBase.isReal()) {
      return new Swerve(
        new RealModule(FRONT_LEFT_DRIVE_ID, FRONT_LEFT_TURN_ID, FRONT_LEFT_ENCODER_ID),
        new RealModule(FRONT_RIGHT_DRIVE_ID, FRONT_RIGHT_TURN_ID, FRONT_RIGHT_ENCODER_ID),
        new RealModule(BACK_RIGHT_DRIVE_ID, BACK_RIGHT_TURN_ID, BACK_RIGHT_ENCODER_ID),
        new RealModule(BACK_LEFT_DRIVE_ID, BACK_LEFT_TURN_ID, BACK_LEFT_ENCODER_ID),
        new NavXGyro()
      );
    } else {
      return new Swerve(
        new SimModule(),
        new SimModule(),
        new SimModule(), 
        new SimModule(),
        new SimGyro()
      );
    }
  }

  /** Represents the orientation of the robot drive. */
  public enum DriveOrientation {
    /** Drives the robot with chassis speeds relative to the robot frame. */
    ROBOT_ORIENTED,

    /** Drives the robot with chassis speeds relative to the field coordinate system. */
    FIELD_ORIENTED
  }

  /** 
   * An odometry thread that updates module status signals at a specified frequency and feeds the signals into a pose estimator.
   */
  public class OdometryThread {
    private Notifier _notifier = new Notifier(this::update);
    private final double _frequency;

    private final BaseStatusSignal[] _signals = new BaseStatusSignal[2 * 4]; // two status signals per module

    public OdometryThread(double frequency) {
      _frequency = frequency;
      _notifier.setName("Odometry Thread");

      if (RobotBase.isSimulation()) return;

      for (int i = 0; i < _modules.size(); i++) {
        BaseStatusSignal[] moduleSignals = ((RealModule) _modules.get(i).getIO()).getOdomSignals();
        _signals[(i*2) + 0] = moduleSignals[0];
        _signals[(i*2) + 1] = moduleSignals[1];
      }

      BaseStatusSignal.setUpdateFrequencyForAll(_frequency, _signals);
    }

    /** Refreshes all the odom status signals, returning true if the action failed. */
    public boolean refreshStatusSignals() {
      if (RobotBase.isReal()) {
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
        willOdomUpdateFail = false; // TODO: figure this out
      }

      // updated all cached device data
      // if a refresh failed, then the cached data would just
      // be from the last successful refresh
      updateCached();
      
      // log everything now instead of the main thread to 
      // reduce logging latency
      log("Module States", getModuleStates());
      log("Desired Module States", getDesiredModuleStates());
      log("Chassis Speeds", getChassisSpeeds());
      log("Desired Chassis Speeds", _desiredChassisSpeeds);
      log("Module Positions", getModulePositions());
      log("Raw Heading", getRawHeading());
      

      // all the devices didn't successfully refresh, so don't update odom
      if (willOdomUpdateFail) {
        _failedOdomUpdates ++;
        _odomUpdateLock.unlock();
        return;
      }

      // all devices refreshed, so update odom with new device data
      _cachedPose = _poseEstimator.update(getRawHeading(), getModulePositions());
      _cachedSimOdomPose = _simOdometry.update(getRawHeading(), getModulePositions());

      log("Robot Pose", getPose()); // log the pose at a higher frequency (also with less latency)
      log("Robot Heading", getHeading());
      // if (RobotBase.isSimulation()) log("Robot Sim Odometry", _cachedSimOdomPose);

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

    if (RobotBase.isSimulation()) {
      _visionSim = new VisionSystemSim("main");
      _visionSim.addAprilTags(FieldConstants.FIELD_LAYOUT);

      _cameras.forEach(cam -> _visionSim.addCamera(cam.getSimCamera(), cam.robotToCam));
    } else {
      _visionSim = null;
    }

    _odomThread = new OdometryThread(SwerveConstants.ODOM_FREQUENCY);
    _odomThread.start();
  }

  /** 
   * Drives the swerve drive.
   * 
   * @param velX The x velocity in meters per second. 
   * @param velY The y velocity in meters per second.
   * @param velOmega The rotational velocity in radians per second.
   */
  public void drive(double velX, double velY, double velOmega) {
    ChassisSpeeds chassisSpeeds;

    if (driveOrientation == DriveOrientation.FIELD_ORIENTED) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        velX,
        velY,
        velOmega,
        getHeading()
      );
    } else {
      chassisSpeeds = new ChassisSpeeds(velX, velY, velOmega);
    }

    chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, Robot.kDefaultPeriod);
    _desiredChassisSpeeds = chassisSpeeds;

    setModuleStates(_kinematics.toSwerveModuleStates(chassisSpeeds));
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
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_TRANSLATIONAL_SPEED.in(MetersPerSecond));

    for (int i = 0; i < _modules.size(); i++) {
      _modules.get(i).setModuleState(states[i], controlMode, allowTurnInPlace);
    }
  }


  // updates the pose estimator with potential vision estimates
  private void updateVisionPoseEstimates() {
    _acceptedEstimates.clear();
    _rejectedEstimates.clear();
    _detectedTargets.clear();

    _cameras.forEach(cam -> {
      var possibleEstimate = cam.getEstimatedPose(_lastestVisionTimestamp);
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
    
    log("Accepted Estimates", _acceptedEstimates.stream().map(VisionPoseEstimate::pose).toArray(Pose2d[]::new));
    log("Rejected Estimates", _rejectedEstimates.stream().map(VisionPoseEstimate::pose).toArray(Pose2d[]::new));

    log("Detected Targets", _detectedTargets.toArray(Pose3d[]::new));

    if (_acceptedEstimates.size() == 0) return;
    _lastestVisionTimestamp = _acceptedEstimates.get(_acceptedEstimates.size() - 1).timestamp();

    // _odomLock.writeLock().lock();
    // _acceptedEstimates.forEach(e -> _poseEstimator.addVisionMeasurement(e.pose(), e.timestamp(), e.stdDevs()));
    // _odomLock.writeLock().unlock();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    for (SwerveModule module : _modules) {
      module.periodic();
    }

    updateVisionPoseEstimates();

    log("Odometry Update Success %", (_attemptedOdomUpdates - _failedOdomUpdates) / _attemptedOdomUpdates * 100.0);
  }

  @Override
  public void simulationPeriodic() {
    _simYaw = _simYaw.plus(Rotation2d.fromRadians(getChassisSpeeds().omegaRadiansPerSecond * Robot.kDefaultPeriod));
    _visionSim.update(_cachedSimOdomPose);
  }

  /** Returns the pose of the drive from the pose estimator. */
  public Pose2d getPose() {
    return _cachedPose;
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
   * Returns the cached raw heading since the last odom update.
   */
  public Rotation2d getRawHeading() {
    return _cachedRawHeading;
  }

  @Override
  public Command selfCheck(BiConsumer<String, AlertType> alerter, BooleanSupplier hasError) {
    return sequentialUntil(
      hasError,
      _frontLeft.selfCheck(alerter, hasError),
      _frontRight.selfCheck(alerter, hasError),
      _backRight.selfCheck(alerter, hasError),
      _backLeft.selfCheck(alerter, hasError)
      // TODO: all other checks
    );
  }
}
