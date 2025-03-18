package frc.util;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants;

/*
 * A strategy that discards tags "too far" from the existing pose
 */
public class VisionFilteringTagDistanceStrategy extends VisionFilteringStrategy {

  public VisionFilteringTagDistanceStrategy(AprilTagFieldLayout fields) {
    super(fields);
  }

  @Override
  public boolean useVisionPose(Pose2d oldPose, Pose2d newVisionPose, EstimatedRobotPose estimatedPose,
      PhotonPipelineResult photonResult) {

    double targetRange = 0.0;
    if (!estimatedPose.targetsUsed.isEmpty()) {
      for (PhotonTrackedTarget target : estimatedPose.targetsUsed) {

        Optional<Pose3d> targetPose = m_fieldLayout.getTagPose(target.fiducialId);
        if (targetPose.isPresent())

          targetRange = calculateRange(oldPose, targetPose.get().toPose2d());
      }
    }
    if (targetRange >= Constants.VisionConstants.kMaxTagRangeM) {
      return false;
    }
    return true;
  }

  private double calculateRange(Pose2d pose1, Pose2d pose2) {
    double result = 0.0;
    result = pose1.getTranslation().getDistance(pose2.getTranslation());
    return result;
  }
}
