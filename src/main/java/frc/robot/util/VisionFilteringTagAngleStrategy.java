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
 * A strategy that discards tags facing us from the existing pose and far away
 * (so we use close tags that are viewed straight-on)
 */
public class VisionFilteringTagAngleStrategy extends VisionFilteringStrategy {

  public VisionFilteringTagAngleStrategy(AprilTagFieldLayout fields) {
    super(fields);
  }

  @Override
  public boolean useVisionPose(Pose2d oldPose, Pose2d newVisionPose, EstimatedRobotPose estimatedPose,
      PhotonPipelineResult photonResult) {

    double targetAngle = 0.0;
    double targetRange = 0.0;
    if (!estimatedPose.targetsUsed.isEmpty()) {
      for (PhotonTrackedTarget target : estimatedPose.targetsUsed) {

        Optional<Pose3d> targetPose = m_fieldLayout.getTagPose(target.fiducialId);
        if (targetPose.isPresent())

          targetAngle = calculateAngle(targetPose.get());
          targetRange = getTagRange(targetPose.get());
      }
    }
    if ((Math.abs(180.0-targetAngle) <= Constants.VisionConstants.kMaxTagAngle) &&
        (targetRange > Constants.VisionConstants.kMaxTagRangeM)) {
      System.out.println("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX Rejecting Tag angle:"+targetAngle +"at range: "+targetRange);
      return false;
    }
    return true;
  }

  // return range in meters
  private double getTagRange(Pose3d target) {
    return target.getX();
  }
  private double calculateAngle(Pose3d target) {
    double result = 0.0;
    Pose2d p2d = target.toPose2d();
    result = p2d.getRotation().getDegrees();
    return result;
  }
}
