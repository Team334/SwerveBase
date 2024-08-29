package frc.robot.util;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;

/** 
 * A class that handles filtering and finding standard deviations for an estimated pose
 * returned from a single photon vision camera.
 */
public class VisionPoseEstimator {
  /**
   * The ambiguity threshold on this camera to limit incoming vision estimates (used for filtering).
   */
  public final double ambiguityThreshold;
  
  /**
   * Std devs factor based on this specific camera, increase it if the resolution is lowered on this camera,
   * or if the fov is high.
   */
  public final double cameraStdDevsFactor;

  /**
   * The location of the camera relative to the robot's center.
   */
  public final Transform3d robotToCam;

  private final PhotonCamera _camera;
  private final PhotonCameraSim _simCamera;

  private final PhotonPoseEstimator _poseEstimator;

  public VisionPoseEstimator(
    String camName, 
    Transform3d robotToCam, 
    double ambiguityThreshold, 
    double cameraStdDevsFactor
  ) {
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
      _simCamera = new PhotonCameraSim(_camera, simProps);
    } else {
      _simCamera = null;
    }
  }

  /**
   * Filters the raw estimate returned from photon vision.
   */
  protected VisionPoseEstimate filterEstimate(EstimatedRobotPose estimate, double latestVisionTimestamp) {
    boolean tooOld = estimate.timestampSeconds < latestVisionTimestamp;

    // and a bunch more filtering

    boolean isValid = tooOld;
    
    return new VisionPoseEstimate(
      estimate.estimatedPose.toPose2d(),
      estimate.timestampSeconds,
      VecBuilder.fill(0, 0, 0),
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
    // first see if camera has any tags in the frame (or if it's even connected)
    Optional<EstimatedRobotPose> rawEstimate = _poseEstimator.update();
    if (rawEstimate.isEmpty()) return Optional.empty();

    // if that worked, filter the estimate, and if the estimate is invalid, just return it as invalid
    VisionPoseEstimate filteredEstimate = filterEstimate(rawEstimate.get(), latestVisionTimestamp);
    if (!filteredEstimate.isValid()) return Optional.of(filteredEstimate);

    // if the estimate is valid, return it with calculated std devs
    return Optional.of(calculateStdDevs(filteredEstimate));
  }

  /** Returns the simulation camera, this is null if the robot isn't being simulated. */
  public PhotonCameraSim getSimCamera() {
    return _simCamera;
  }
}
