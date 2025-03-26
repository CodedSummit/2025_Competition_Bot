package frc.robot.util;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/*
 * A strategy that discards tags "too far" from the existing pose
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

          targetAngle = calculateAngle(targetPose.get());
      }
    }
    if (Math.abs(180.0-targetAngle) >= Constants.VisionConstants.kMaxTagAngle) {
      System.out.println("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX Rejecting Tag angle:"+targetAngle);
      return false;
    }
    return true;
  }

  private double calculateAngle(Pose3d target) {
    double result = 0.0;
    Pose2d p2d = target.toPose2d();
    result = p2d.getRotation().getDegrees();
    return result;
  }
}
