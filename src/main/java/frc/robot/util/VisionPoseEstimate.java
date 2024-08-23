package frc.robot.util;

import java.nio.ByteBuffer;
import java.util.Comparator;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

/** Represents a vision pose estimate to feed into the pose estimator. */
public record VisionPoseEstimate(
  Pose2d pose,
  double timestamp,
  Vector<N3> stdDevs
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
      return Pose2d.struct.getSize() + kSizeDouble * 4;
    }

    @Override
    public String getSchema() {
      return "Pose2d pose;double timestamp;double xStdDev;double yStdDev;double thetaStdDev;";
    }

    @Override
    public VisionPoseEstimate unpack(ByteBuffer bb) {
      Pose2d pose = Pose2d.struct.unpack(bb);
      double timestamp = bb.getDouble();
      double xStdDev = bb.getDouble();
      double yStdDev = bb.getDouble();
      double thetaStdDev = bb.getDouble();

      return new VisionPoseEstimate(pose, timestamp, VecBuilder.fill(xStdDev, yStdDev, thetaStdDev));
    }

    @Override
    public void pack(ByteBuffer bb, VisionPoseEstimate value) {
      Pose2d.struct.pack(bb, value.pose);
      bb.putDouble(value.timestamp);
      bb.putDouble(value.stdDevs.get(0, 0));
      bb.putDouble(value.stdDevs.get(1, 0));
      bb.putDouble(value.stdDevs.get(2, 0));
    }
  }
};

