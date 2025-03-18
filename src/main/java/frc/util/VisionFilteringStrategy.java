package frc.util;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;

/*
 * A classs to determine whether to use a given vision measurement or not
 */
public class VisionFilteringStrategy {
  protected AprilTagFieldLayout m_fieldLayout;
  
  public  VisionFilteringStrategy(AprilTagFieldLayout fields){
      m_fieldLayout = fields;
  }

  private VisionFilteringStrategy() {} // force use of tag constructor
  /*
   * Returns true if the provided vision pose should be used, else false
   * Derived classes should override to implement differnet strategies.
   * Base implementation approves all new poses (i.e. always returns true)
   */
  
  public boolean useVisionPose(Pose2d oldPose, Pose2d newVisionPose, EstimatedRobotPose estimatedPose, PhotonPipelineResult photonResults) {
    return true;
  }
}
