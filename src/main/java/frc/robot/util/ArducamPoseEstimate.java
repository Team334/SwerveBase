package frc.robot.util;

import java.nio.ByteBuffer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

/** Represents a pose estimate from an arducam to feed into the pose estimator. */
public record ArducamPoseEstimate(
  Pose2d pose,
  double timestamp,
  double xStdDev,
  double yStdDev,
  double thetaStdDev
) implements StructSerializable {
  public final static ArducamPoseEstimateStruct struct = new ArducamPoseEstimateStruct();

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

      return new ArducamPoseEstimate(pose, timestamp, xStdDev, yStdDev, thetaStdDev);
    }

    @Override
    public void pack(ByteBuffer bb, ArducamPoseEstimate value) {
      Pose2d.struct.pack(bb, value.pose);
      bb.putDouble(value.timestamp);
      bb.putDouble(value.xStdDev);
      bb.putDouble(value.yStdDev);
      bb.putDouble(value.thetaStdDev);
    }
  }
};

