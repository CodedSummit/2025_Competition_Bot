package frc.robot.util;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants;

/*
 * A strategy that discards tags when facing straight-on (leading to ambiguity and jitter in yaw)
 */
public class VisionFilteringTagAngleStrategy extends VisionFilteringStrategy {

  public VisionFilteringTagAngleStrategy(AprilTagFieldLayout fields) {
    super(fields);
  }

  @Override
  public boolean useVisionPose(Pose2d oldPose, Pose2d newVisionPose, EstimatedRobotPose estimatedPose,
      PhotonPipelineResult photonResult) {

    double targetAngle = 0.0;
    if (!estimatedPose.targetsUsed.isEmpty()) {
      for (PhotonTrackedTarget target : estimatedPose.targetsUsed) {

        Optional<Pose3d> targetPose = m_fieldLayout.getTagPose(target.fiducialId);
        if (targetPose.isPresent())

          targetAngle = calculateAngle(oldPose, targetPose.get().toPose2d());
      }
    }
    if (Math.abs(180.0 - targetAngle) >= Constants.VisionConstants.kMaxTagRangeM) {
      return false;
    }
    return true;
  }

  private double calculateAngle(Pose2d pose1, Pose2d pose2) {
    double result = 0.0;
    result = pose2.getRotation().getDegrees();
    return result;
  }
}
