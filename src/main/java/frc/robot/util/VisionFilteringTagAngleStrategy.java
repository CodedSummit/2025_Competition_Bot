package frc.robot.util;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
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

    if (oldPose.getX() < 1.0 && oldPose.getY() < 1.0 ){
      return true;   // we have no pose yet so use what we get
    }  
    if ((oldPose == null) || (newVisionPose == null) || (estimatedPose == null)) {
      return false;  // avoid catastrophic failures
    }  
    double targetToBotAngle = 0.0;
    double targetRange = 0.0;
    Pose2d targetPose2d = null;
    if (!estimatedPose.targetsUsed.isEmpty()) {
      for (PhotonTrackedTarget target : estimatedPose.targetsUsed) {

        Optional<Pose3d> targetPose = m_fieldLayout.getTagPose(target.fiducialId);
        if (targetPose.isPresent())
          targetPose2d = targetPose.get().toPose2d();
          targetToBotAngle = getTagToBotAngle(oldPose, targetPose2d);
          targetRange = getTagRange(oldPose, targetPose.get().toPose2d());
      }
    }
    System.out.println("     Target to bot angle:"+targetToBotAngle+"  range:"+targetRange + "  Target rot:"+targetPose2d.getRotation().getDegrees());
    if ((Math.abs(targetToBotAngle - Math.abs(180-targetPose2d.getRotation().getDegrees())) <= Constants.VisionConstants.kMaxTagAngle) &&
        (targetRange > Constants.VisionConstants.kMaxTagRangeM)) {
      System.out.println("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX Rejecting Tag angle:"+targetToBotAngle +"at range: "+targetRange);
      return false;
    }
    return true;
  }

  /*
   * return range in meters
   */
  private double getTagRange(Pose2d me, Pose2d tag) {
    double range =0.0;
    Pose2d diff = me.relativeTo(tag);
    range = Math.sqrt(diff.getX()*diff.getX() + diff.getY()*diff.getY());
    return range;
  }
  
  /* return angle, in 0-180 range
   * 
   */
  private double getTagToBotAngle (Pose2d bot, Pose2d tag){
    Transform2d xform = bot.minus(tag);
    return calculateAngle(xform.getX(), xform.getY());
  }
  private double calculateAngle(double x, double y) {
    double result = 0.0;
    result = Math.toDegrees(Math.atan(y/x));
    return result;
  }
}
