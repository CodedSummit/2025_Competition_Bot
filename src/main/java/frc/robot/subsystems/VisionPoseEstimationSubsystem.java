// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
/**
 *  Uses Vision data to come up with estimated poses that can be fed to the drive system pose estimator.
 *  Don't really need this to be a subsystem since it's doesn't need mutex protection - but keeping it as one fits 
 *  the design pattern.
 * Uses the Left and Right cameras for estimations.
 */
public class VisionPoseEstimationSubsystem extends SubsystemBase {
 
  PhotonCamera m_backCamera = new PhotonCamera(VisionConstants.kBackCamName);
  PhotonCamera m_leftCamera = new PhotonCamera(VisionConstants.kLeftCamName);
  PhotonCamera m_rightCamera = new PhotonCamera(VisionConstants.kRightCamName);
  AprilTagFieldLayout m_CompetitionAprilTagFieldLayout = AprilTagFields.k2025ReefscapeWelded.loadAprilTagLayoutField();
  PhotonPoseEstimator m_backCamPhotonPoseEstimator=null;
  PhotonPoseEstimator m_leftCamPhotonPoseEstimator = null;
  PhotonPoseEstimator m_rightCamPhotonPoseEstimator = null;
  private boolean m_visionEnabled = true;
  SwerveSubsystem m_SwerveSubsystem;
   private IntegerLogEntry m_rightLog;
   private IntegerLogEntry m_leftLog;
   private IntegerLogEntry m_backLog;
   private IntegerLogEntry m_targetIds;
  private long m_lastLogTime = 0;

  /** Creates a new VisionPoseEstimationSubsystem. */
  public VisionPoseEstimationSubsystem() {

 

    // Construct PhotonPoseEstimators
     m_backCamPhotonPoseEstimator = new PhotonPoseEstimator(m_CompetitionAprilTagFieldLayout, 
      PoseStrategy.AVERAGE_BEST_TARGETS, VisionConstants.kRobotToBackCam);
    m_leftCamPhotonPoseEstimator = new PhotonPoseEstimator(m_CompetitionAprilTagFieldLayout,
        PoseStrategy.AVERAGE_BEST_TARGETS, VisionConstants.kRobotToLeftCam);
    m_rightCamPhotonPoseEstimator = new PhotonPoseEstimator(m_CompetitionAprilTagFieldLayout,
        PoseStrategy.AVERAGE_BEST_TARGETS, VisionConstants.kRobotToRightCam);
    DataLog log = DataLogManager.getLog();
    m_rightLog = new IntegerLogEntry(log, "RightCamTargets");
    m_leftLog = new IntegerLogEntry(log, "LeftCamTargets");
    m_backLog = new IntegerLogEntry(log, "BackCamTargets");
    m_targetIds = new IntegerLogEntry(log, "TargetIDs");

    initialize();
  }
    
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    logVisionTargets();
  }

  public void initialize() {
     ShuffleboardTab vTab = Shuffleboard.getTab("Vision");
     vTab.addBoolean("Enable vision ", () -> enableVisionPose(m_visionEnabled))
     .withWidget(BuiltInWidgets.kToggleSwitch)
     .withSize(1,1);

  }
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
 /**
  * Log the targets seen by the cameras for later analysis
  */
  public void logVisionTargets() {
    // change to keep a local cache of targets seen, and periodically write them out
    long t = System.currentTimeMillis();
    if (VisionConstants.kLogInterval != 0 && (t - m_lastLogTime > VisionConstants.kLogInterval)) {
      m_lastLogTime = t;

      PhotonPipelineResult result = m_rightCamera.getLatestResult();
      List<PhotonTrackedTarget> targets = result.getTargets();
      if (result.hasTargets()) {
        m_rightLog.append(targets.size());
        targets.forEach(id -> m_targetIds.append(id.getFiducialId()));
      }
      else {
        m_rightLog.append(0);
      }
      result = m_leftCamera.getLatestResult();
      targets = result.getTargets();
      if (result.hasTargets()) {
        m_leftLog.append(targets.size());
        targets.forEach(id -> m_targetIds.append(id.getFiducialId()));
      }
      else {
        m_leftLog.append(0);
      }
      result = m_backCamera.getLatestResult();
      targets = result.getTargets();
      if (result.hasTargets()) {
        m_backLog.append(targets.size());
        targets.forEach(id -> m_targetIds.append(id.getFiducialId()));
      }
      else {
        m_backLog.append(0);
      }
    }
  }


  //  Get the actual estimates from the different cameras (Left (LC), Right (RC), back (BC))
  private Optional<EstimatedRobotPose> getLCEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
      m_leftCamPhotonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
      PhotonPipelineResult result = m_leftCamera.getLatestResult();
      return m_leftCamPhotonPoseEstimator.update(result);
  }
 private Optional<EstimatedRobotPose> getRCEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
      m_rightCamPhotonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
      PhotonPipelineResult result = m_rightCamera.getLatestResult();
      return m_rightCamPhotonPoseEstimator.update(result);
  }
  private Optional<EstimatedRobotPose> getBCEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
      m_backCamPhotonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
      PhotonPipelineResult result = m_backCamera.getLatestResult();
      return m_backCamPhotonPoseEstimator.update(result);
  }

  /**
   * Adds vision esimtates to the provided pose esimator.
   * If vision pose estimation is enabled, get estimates from both right and left cameras and mix them in
   * to the estimation.
   * 
   * @param poseEstimator
   */

  public void updatePoseWithVision(SwerveDrivePoseEstimator poseEstimator) {

    boolean received_vision_update = false;
    if (getVisionEnable()) {

      var pose = getLCEstimatedGlobalPose(poseEstimator.getEstimatedPosition());
      if (pose.isPresent()) {
        var pose2d = pose.get().estimatedPose.toPose2d();
        poseEstimator.addVisionMeasurement(pose2d, pose.get().timestampSeconds);
        received_vision_update = true;
 //       System.out.println(" Updated pose with left cam vision.  x:" + pose2d.getX() + "   y: " + pose2d.getY());
      }
      pose = getRCEstimatedGlobalPose(poseEstimator.getEstimatedPosition());
      if (pose.isPresent()) {
        var pose2d = pose.get().estimatedPose.toPose2d();
        poseEstimator.addVisionMeasurement(pose2d, pose.get().timestampSeconds);
 //       System.out.println(" Updated pose with right cam vision.  x:" + pose2d.getX() + "   y: " + pose2d.getY());
        received_vision_update = true;
      }
      pose = getBCEstimatedGlobalPose(poseEstimator.getEstimatedPosition());
      if (pose.isPresent()) {
        var pose2d = pose.get().estimatedPose.toPose2d();
        poseEstimator.addVisionMeasurement(pose2d, pose.get().timestampSeconds);
  //      System.out.println(" Updated pose with back cam vision.  x:" + pose2d.getX() + "   y: " + pose2d.getY());
        received_vision_update = true;
      }

    }
    if(received_vision_update){
      //m_led.setEndsGreen();

    } else {
      //m_led.setEndsOff();
    }

  }

  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("Vision Location Enabled", this::getVisionEnable, this::enableVisionPose);
  }

  public boolean getVisionEnable(){
    return m_visionEnabled;
  }
  public boolean enableVisionPose(boolean b) {
    m_visionEnabled = b;
    return m_visionEnabled;
  }

}
