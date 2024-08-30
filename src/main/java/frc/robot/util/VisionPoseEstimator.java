package frc.robot.util;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
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
  protected void filterEstimate(EstimatedRobotPose estimate, double latestVisionTimestamp) {
    boolean tooOld = estimate.timestampSeconds <= latestVisionTimestamp;
    
    // and a bunch more filtering

    boolean isValid = !tooOld;
    // boolean isValid = true;

    int[] detectedTags = estimate.targetsUsed.stream().mapToInt(PhotonTrackedTarget::getFiducialId).toArray();
    
    _latestEstimate = new VisionPoseEstimate(
      estimate.estimatedPose.toPose2d(),
      estimate.timestampSeconds,
      detectedTags,
      _latestEstimate.stdDevs,
      isValid
    );
  }

  /**
   * Calculates the standard deviations for the given filtered estimate.
   */
  protected VisionPoseEstimate calculateStdDevs(VisionPoseEstimate estimate) {
    return estimate;
  }

  /**
   * Returns an optional containing the vision pose estimate, if no tags were seen this optional will be empty.
   */
  public Optional<VisionPoseEstimate> getEstimatedPose(double latestVisionTimestamp) {
    _latestEstimate = VisionPoseEstimate.noDetectedTags();

    // first see if camera has any tags in the frame (or if it's even connected)
    Optional<EstimatedRobotPose> rawEstimate = _poseEstimator.update();
    if (rawEstimate.isEmpty()) return Optional.empty();

    // if that worked, filter the estimate, and if the estimate is invalid, just return it as invalid
    filterEstimate(rawEstimate.get(), latestVisionTimestamp);
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
    log("Detected Tags", _latestEstimate.detectedTags);
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
    String camName, 
    Transform3d robotToCam, 
    double ambiguityThreshold, 
    double cameraStdDevsFactor
  ) {};

  /** Represents a vision pose estimate. */
  public record VisionPoseEstimate(
    Pose2d pose,
    double timestamp,
    int[] detectedTags,
    Vector<N3> stdDevs,
    boolean isValid
  ) {
    /** 
     * Returns a vision pose estimate that represents an estimate with no 
     * detected tags (or camera was disconnected).
     */
    public final static VisionPoseEstimate noDetectedTags() {
      return new VisionPoseEstimate(
        new Pose2d(),
        -1,
        new int[0],
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
