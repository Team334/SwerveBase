package frc.robot.util;

import java.util.ArrayDeque;
import java.util.Deque;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class OdomDemo {
  private final Deque<Pose2d> _odomPoses = new ArrayDeque<>(1);
  private Pose2d _lastPose = new Pose2d(); 
  
  private final SwerveDriveKinematics _kinematics;

  private SwerveModulePosition[] _lastPositions;
  private Rotation2d _lastHeading;

  public OdomDemo(SwerveDriveKinematics kinematics) {
    _kinematics = kinematics;
  }

  public Pose2d[] getPoses() {
    return _odomPoses.toArray(Pose2d[]::new);
  }

  public void reset(Pose2d pose, SwerveModulePosition[] modulePositions, Rotation2d heading) {
    _lastPose = pose;
    _lastPositions = modulePositions;
    _lastHeading = heading;
  }

  public Pose2d[] update(SwerveModulePosition[] modulePositions, Rotation2d heading) {
    SwerveModulePosition[] deltas = new SwerveModulePosition[4];

    for (int i = 0; i < 4; i++) {
      deltas[i] = new SwerveModulePosition(
        modulePositions[i].distanceMeters - _lastPositions[i].distanceMeters,
        modulePositions[i].angle.minus(_lastPositions[i].angle)
      );
    }

    var twist = _kinematics.toTwist2d(deltas);
    twist.dtheta = heading.minus(_lastHeading).getRadians();

    var newPose = _lastPose.exp(twist);

    _lastPositions = modulePositions;
    _lastHeading = heading;
    _lastPose = new Pose2d(newPose.getTranslation(), heading);

    _odomPoses.offerFirst(_lastPose);

    if (_odomPoses.size() > 10) {
      _odomPoses.removeLast();
    }

    return getPoses();
  }
}
