package frc.robot.util;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import monologue.Logged;
import monologue.Annotations.Log;

/** 
 * A class that handles filtering and finding standard deviations for an estimated pose
 * returned from a single photon vision camera.
 */
public class VisionPoseEstimator implements Logged {
  private VisionPoseEstimate _latestEstimate;

  private final PhotonCamera _camera;
  private final PhotonCameraSim _simCamera;

  private final PhotonPoseEstimator _poseEstimator;

  /**
   * The camera's NT name.
   */
  @Log.NT.Once(key = "Camera Name")
  public final String camName;

  /**
   * The ambiguity threshold on this camera to limit incoming vision estimates (used for filtering).
   */
  @Log.NT.Once(key = "Ambiguity Threshold")
  public final double ambiguityThreshold;
  
  /**
   * Std devs factor based on this specific camera, increase it if the resolution is lowered on this camera,
   * if the fov is high, if the ambiguity threshold is increased, etc.
   */
  @Log.NT.Once(key = "Camera Std Devs Factor")
  public final double cameraStdDevsFactor;

  /**
   * The location of the camera relative to the robot's center.
   */
  @Log.NT.Once(key = "Robot To Cam Transform")
  public final Transform3d robotToCam;

  /**
   * Whether this estimator is ignoring the vision heading estimate (if this is true the vision theta std devs will be super high).
   */
  @Log.NT(key = "Ignore Theta Estimate")
  public final boolean ignoreThetaEstimate = true;

  /**
   * Builds a new vision pose estimator from a single camera constants.
   */
  public static VisionPoseEstimator buildFromCamera(VisionPoseEstimatorConstants camConstants) {
    return new VisionPoseEstimator(
      camConstants.camName,
      camConstants.robotToCam,
      camConstants.ambiguityThreshold,
      camConstants.cameraStdDevsFactor
    );
  }

  /**
   * Builds multiple vision pose estimators from a list of camera constants.
   * 
   * @return An array of estimators.
   */
  public static List<VisionPoseEstimator> buildFromCameras(List<VisionPoseEstimatorConstants> allConstants) {
    List<VisionPoseEstimator> estimators = new ArrayList<VisionPoseEstimator>();

    allConstants.forEach(camConstants -> {
      estimators.add(buildFromCamera(camConstants));
    });

    return estimators;
  }

  /**
   * Creates a new VisionPoseEstimator (all params are members that are javadocced already).
   */
  public VisionPoseEstimator(
    String camName, 
    Transform3d robotToCam, 
    double ambiguityThreshold, 
    double cameraStdDevsFactor
  ) {
    this.camName = camName;
    this.robotToCam = robotToCam;
    this.ambiguityThreshold = ambiguityThreshold;
    this.cameraStdDevsFactor = cameraStdDevsFactor;

    _camera = new PhotonCamera(camName);

    _poseEstimator = new PhotonPoseEstimator(
      Constants.FieldConstants.FIELD_LAYOUT, 
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      _camera,
      robotToCam
    );

    _poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  
    if (RobotBase.isSimulation()) {
      SimCameraProperties simProps = new SimCameraProperties();

      simProps.setCalibError(0.01, 0.005);

      _simCamera = new PhotonCameraSim(_camera, simProps);
    } else {
      _simCamera = null;
    }
  }

  /**
   * Filters the raw estimate returned from photon vision.
   */
  protected void filterEstimate(EstimatedRobotPose estimate, double latestVisionTimestamp, Rotation2d gyroHeading) {
    // get all info from the estimate
    Pose3d estimatedPose = estimate.estimatedPose;
    double timestamp = estimate.timestampSeconds;
    double ambiguity = -1;
    int tagAmount = estimate.targetsUsed.size();
    int[] detectedTags = new int[tagAmount];
    double avgTagDistance = 0;

    // disambiguate poses using gyro measurement (only necessary for a single tag)
    if (tagAmount == 1) {
      PhotonTrackedTarget target = estimate.targetsUsed.get(0);
      int tagId = target.getFiducialId();
      Pose3d tagPose = FieldConstants.FIELD_LAYOUT.getTagPose(tagId).get();
      
      ambiguity = target.getPoseAmbiguity();

      Pose3d betterReprojPose = tagPose.transformBy(target.getBestCameraToTarget().inverse());
      Pose3d worseReprojPose = tagPose.transformBy(target.getAlternateCameraToTarget().inverse());

      betterReprojPose = betterReprojPose.transformBy(robotToCam.inverse());
      worseReprojPose = worseReprojPose.transformBy(robotToCam.inverse());

      // check which of the poses is closer to the correct gyro heading
      if (
        Math.abs(betterReprojPose.toPose2d().getRotation().minus(gyroHeading).getDegrees()) <
        Math.abs(worseReprojPose.toPose2d().getRotation().minus(gyroHeading).getDegrees())
      ) {
        estimatedPose = betterReprojPose;
        log("USED BETTER", true);
      } else {
        estimatedPose = worseReprojPose;
        log("USED BETTER", false);
      }

      log("Angles", "Better " + Double.toString(betterReprojPose.toPose2d().getRotation().getDegrees()) + "Worse " + Double.toString(worseReprojPose.toPose2d().getRotation().getDegrees()));
      log("Gyro Heading @ T", gyroHeading.getDegrees());
    }

    // get tag distance
    for (int i = 0; i < tagAmount; i++) {
      int tagId = estimate.targetsUsed.get(i).getFiducialId();
      Pose3d tagPose = FieldConstants.FIELD_LAYOUT.getTagPose(tagId).get();
      detectedTags[i] = tagId;
      avgTagDistance += tagPose.getTranslation().getDistance(
        estimatedPose.getTranslation()
      );
    }

    avgTagDistance /= tagAmount;


    // run all filtering
    boolean tooOld = timestamp <= latestVisionTimestamp;
    boolean badAmbiguity = ambiguity >= ambiguityThreshold;
    boolean outOfBounds = (
      estimatedPose.getX() <= -VisionConstants.X_BOUND_MARGIN ||
      estimatedPose.getX() >= FieldConstants.FIELD_LAYOUT.getFieldLength() + VisionConstants.X_BOUND_MARGIN ||
      estimatedPose.getY() <= -VisionConstants.Y_BOUND_MARGIN ||
      estimatedPose.getY() >= FieldConstants.FIELD_LAYOUT.getFieldWidth() + VisionConstants.Y_BOUND_MARGIN ||
      estimatedPose.getZ() >= VisionConstants.Z_BOUND_MARGIN ||
      estimatedPose.getZ() <= -VisionConstants.Z_BOUND_MARGIN
    );
    
    boolean isValid = !(tooOld || badAmbiguity || outOfBounds);
    // boolean isValid = true;
    
    _latestEstimate = new VisionPoseEstimate(
      estimatedPose,
      timestamp,
      ambiguity,
      detectedTags,
      avgTagDistance,
      _latestEstimate.stdDevs,
      isValid
    );
  }

  /**
   * Calculates the standard deviations for the given filtered estimate.
   */
  protected void calculateStdDevs(VisionPoseEstimate estimate) {
    Vector<N3> baseStdDevs = estimate.detectedTags.length == 1 ? 
      VisionConstants.SINGLE_TAG_BASE_STDDEVS :
      VisionConstants.MULTI_TAG_BASE_STDDEVS;

    double xStdDevs = baseStdDevs.get(0, 0) * Math.pow(estimate.avgTagDistance, 2) * cameraStdDevsFactor;
    double yStdDevs = baseStdDevs.get(1, 0) * Math.pow(estimate.avgTagDistance, 2) * cameraStdDevsFactor;
    double thetaStdDevs = baseStdDevs.get(2, 0) * Math.pow(estimate.avgTagDistance, 2) * cameraStdDevsFactor;
    
    if (ignoreThetaEstimate) thetaStdDevs = 999999999;

    _latestEstimate = new VisionPoseEstimate(
      estimate.pose,
      estimate.timestamp,
      estimate.ambiguity,
      estimate.detectedTags,
      estimate.avgTagDistance,
      VecBuilder.fill(xStdDevs, yStdDevs, thetaStdDevs),
      estimate.isValid
    );
  }

  /**
   * Returns an optional containing the vision pose estimate, if no tags were seen this optional will be empty.
   * 
   * @param latestVisionTimestamp The timestamp of the latest vision estimate used in the swerve pose estimator.
   * @param gyroHeadingAtTime Function to get the heading of the gyro at a timestamp, used to disambiguate single tag estimates.
   */
  public Optional<VisionPoseEstimate> getEstimatedPose(double latestVisionTimestamp, Function<Double, Rotation2d> gyroHeadingAtTime) {
    // start with no detected tags
    _latestEstimate = VisionPoseEstimate.noDetectedTags();

    // first see if camera has any tags in the frame (or if it's even connected)
    Optional<EstimatedRobotPose> rawEstimate = _poseEstimator.update();
    if (rawEstimate.isEmpty()) return Optional.empty();

    // if that worked, filter the estimate, and if the estimate is invalid, just return it as invalid
    filterEstimate(rawEstimate.get(), latestVisionTimestamp, gyroHeadingAtTime.apply(rawEstimate.get().timestampSeconds));
    if (!_latestEstimate.isValid()) return Optional.of(_latestEstimate);

    // if the estimate is valid, return it with calculated std devs
    calculateStdDevs(_latestEstimate);
    return Optional.of(_latestEstimate);
  }

  /**
   * Logs the latest estimate since the last {@link #getEstimatedPose} call.
   */
  public void logLatestEstimate() {
    log("Pose", _latestEstimate.pose);
    log("Timestamp", _latestEstimate.timestamp);
    log("Ambiguity", _latestEstimate.ambiguity);
    log("Detected Tags", _latestEstimate.detectedTags);
    log("Average Tag Distance", _latestEstimate.avgTagDistance);
    log("X Std Devs", _latestEstimate.stdDevs.get(0, 0));
    log("Y Std Devs", _latestEstimate.stdDevs.get(1, 0));
    log("Theta Std Devs", _latestEstimate.stdDevs.get(2, 0));
    log("Is Valid", _latestEstimate.isValid);
  }

  /** Returns the simulation camera, this is null if the robot isn't being simulated. */
  public PhotonCameraSim getSimCamera() {
    return _simCamera;
  }

  /**
   * Constants for a single vision pose estimator camera.
   */
  public record VisionPoseEstimatorConstants(
    /** The NT name of the camera. */
    String camName, 

    /** The robot to camera transform */
    Transform3d robotToCam,

    /** The ambiguity threshold for filtering */
    double ambiguityThreshold,

    /** The camera's std devs factor. */
    double cameraStdDevsFactor
  ) {};

  /** Represents a vision pose estimate. */
  public record VisionPoseEstimate(
    /** The pose to add into the estimator. */
    Pose3d pose,

    /** The timestamp of when the frame was taken. (-1 when no tags). */
    double timestamp,

    /** The ambiguity of this measurement (-1 when no tags or when multi-tag). */
    double ambiguity,

    /** The detected tag ids in this measurement. */
    int[] detectedTags,

    /** The average distance from the tag(s) (-1 when no tags). */
    double avgTagDistance,

    /** The [xMeters, yMeters, thetaRadians] noise standard deviations of this pose estimate ([-1, -1, -1] when no tags or invalid). */
    Vector<N3> stdDevs,

    /** Whether this estimate passed the filter or not. */
    boolean isValid
  ) {
    /** 
     * Returns a vision pose estimate that represents an estimate with no 
     * detected tags (or camera was disconnected).
     */
    public final static VisionPoseEstimate noDetectedTags() {
      return new VisionPoseEstimate(
        new Pose3d(),
        -1,
        -1,
        new int[0],
        -1,
        VecBuilder.fill(-1, -1, -1),
        false
      );
    }

    /**
     * Used for sorting a list of arducam pose estimates, first the timestamps are sorted,
     * then the standard deviations are sorted (based on which standard deviation is overall better).
     */
    public final static Comparator<VisionPoseEstimate> comparator = Comparator.comparing(
      VisionPoseEstimate::timestamp,
      (t1, t2) -> {
        if (t1 > t2) return 1;
        if (t1 < t2) return -1;
        return 0;
      }
    ).thenComparing(
      VisionPoseEstimate::stdDevs,
      (s1, s2) -> {
        return -Double.compare(
          s1.get(0, 0) + s1.get(1, 0) + s1.get(2, 0),
          s2.get(0, 0) + s2.get(1, 0) + s2.get(2, 0)
        );
      }
    );
  };
}
