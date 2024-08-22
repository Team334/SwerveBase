package frc.robot.util;

import java.nio.ByteBuffer;
import java.util.Comparator;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

/** Represents a pose estimate from an arducam to feed into the pose estimator. */
public record ArducamPoseEstimate(
  Pose2d pose,
  double timestamp,
  Vector<N3> stdDevs
) implements StructSerializable {
  public final static ArducamPoseEstimateStruct struct = new ArducamPoseEstimateStruct();

  /**
   * Used for sorting a list of arducam pose estimates, first the timestamps are sorted,
   * then the standard deviations are sorted (based on which standard deviation is overall better).
   */
  public final static Comparator<ArducamPoseEstimate> comparator = Comparator.comparing(
    ArducamPoseEstimate::timestamp,
    (t1, t2) -> {
      if (t1 > t2) return 1;
      if (t1 < t2) return -1;
      return 0;
    }
  ).thenComparing(
    ArducamPoseEstimate::stdDevs,
    (s1, s2) -> {
      return -Double.compare(
        s1.get(0, 0) + s1.get(1, 0) + s1.get(2, 0),
        s2.get(0, 0) + s2.get(1, 0) + s2.get(2, 0)
      );
    }
  );

  private static class ArducamPoseEstimateStruct implements Struct<ArducamPoseEstimate> {
    @Override
    public Class<ArducamPoseEstimate> getTypeClass() {
      return ArducamPoseEstimate.class;
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
    public ArducamPoseEstimate unpack(ByteBuffer bb) {
      Pose2d pose = Pose2d.struct.unpack(bb);
      double timestamp = bb.getDouble();
      double xStdDev = bb.getDouble();
      double yStdDev = bb.getDouble();
      double thetaStdDev = bb.getDouble();

      return new ArducamPoseEstimate(pose, timestamp, VecBuilder.fill(xStdDev, yStdDev, thetaStdDev));
    }

    @Override
    public void pack(ByteBuffer bb, ArducamPoseEstimate value) {
      Pose2d.struct.pack(bb, value.pose);
      bb.putDouble(value.timestamp);
      bb.putDouble(value.stdDevs.get(0, 0));
      bb.putDouble(value.stdDevs.get(1, 0));
      bb.putDouble(value.stdDevs.get(2, 0));
    }
  }
};

