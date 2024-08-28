package frc.robot.util;

import java.nio.ByteBuffer;
import java.util.Comparator;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
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
    if (!filteredEstimate.isValid) return Optional.of(filteredEstimate);

    // if the estimate is valid, return it with calculated std devs
    return Optional.of(calculateStdDevs(filteredEstimate));
  }

  /** Returns the simulation camera, this is null if the robot isn't being simulated. */
  public PhotonCameraSim getSimCamera() {
    return _simCamera;
  }

  /** Represents a vision pose estimate. */
  public record VisionPoseEstimate(
    Pose2d pose,
    double timestamp,
    Vector<N3> stdDevs,
    boolean isValid
  ) implements StructSerializable {
    public final static VisionPoseEstimateStruct struct = new VisionPoseEstimateStruct();

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

    private static class VisionPoseEstimateStruct implements Struct<VisionPoseEstimate> {
      @Override
      public Class<VisionPoseEstimate> getTypeClass() {
        return VisionPoseEstimate.class;
      }

      @Override
      public String getTypeString() {
        return "struct:ArducamPoseEstimate";
      }

      @Override
      public int getSize() {
        return Pose2d.struct.getSize() + kSizeDouble * 4 + kSizeInt8;
      }

      @Override
      public String getSchema() {
        return "Pose2d pose;double timestamp;double xStdDev;double yStdDev;double thetaStdDev;int passedFilter;";
      }

      @Override
      public VisionPoseEstimate unpack(ByteBuffer bb) {
        Pose2d pose = Pose2d.struct.unpack(bb);
        double timestamp = bb.getDouble();
        double xStdDev = bb.getDouble();
        double yStdDev = bb.getDouble();
        double thetaStdDev = bb.getDouble();
        boolean isValid = bb.getInt() == 1 ? true : false;

        return new VisionPoseEstimate(pose, timestamp, VecBuilder.fill(xStdDev, yStdDev, thetaStdDev), isValid);
      }

      @Override
      public void pack(ByteBuffer bb, VisionPoseEstimate value) {
        Pose2d.struct.pack(bb, value.pose);
        bb.putDouble(value.timestamp);
        bb.putDouble(value.stdDevs.get(0, 0));
        bb.putDouble(value.stdDevs.get(1, 0));
        bb.putDouble(value.stdDevs.get(2, 0));
        bb.putInt(value.isValid ? 1 : 0);
      }
    }
  };
}
