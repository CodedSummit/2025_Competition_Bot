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
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.VisionFilteringStrategy;
import frc.robot.util.VisionFilteringTagDistanceStrategy;
/**
 *  Uses Vision data to come up with estimated poses that can be fed to the drive system pose estimator.
 *  Don't really need this to be a subsystem since it's doesn't need mutex protection - but keeping it as one fits 
 *  the design pattern.
 * Uses the Left and Right cameras for estimations.
 */
@Logged
public class VisionPoseEstimationSubsystem extends SubsystemBase {
 
  PhotonCamera m_backCamera = new PhotonCamera(VisionConstants.kBackCamName);
  PhotonCamera m_frontCamera = new PhotonCamera(VisionConstants.kFrontCamName);
  PhotonCamera m_rightCamera = new PhotonCamera(VisionConstants.kRightCamName);
  AprilTagFieldLayout m_CompetitionAprilTagFieldLayout = AprilTagFields.k2025ReefscapeWelded.loadAprilTagLayoutField();
  PhotonPoseEstimator m_backCamPhotonPoseEstimator=null;
  PhotonPoseEstimator m_frontCamPhotonPoseEstimator = null;
  PhotonPoseEstimator m_rightCamPhotonPoseEstimator = null;
  StructPublisher<Pose2d> m_backcamPub;
  StructPublisher<Pose2d> m_frontcamPub;
  StructPublisher<Pose2d> m_rightcamPub;

  
  private boolean m_visionEnabled = true;
   private IntegerLogEntry m_rightLog;
   private IntegerLogEntry m_frontLog;
   private IntegerLogEntry m_backLog;
   private IntegerLogEntry m_targetIds;
  private long m_lastLogTime = 0;

  private GenericEntry nt_visionEnabled;

  private VisionFilteringStrategy m_visionFilter=null;

  /** Creates a new VisionPoseEstimationSubsystem. */
  public VisionPoseEstimationSubsystem() {

 
    // Construct PhotonPoseEstimators
    m_backCamPhotonPoseEstimator = new PhotonPoseEstimator(m_CompetitionAprilTagFieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.kRobotToBackCam);
    m_backCamPhotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    m_frontCamPhotonPoseEstimator = new PhotonPoseEstimator(m_CompetitionAprilTagFieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.kRobotToFrontCam);
    m_frontCamPhotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    m_rightCamPhotonPoseEstimator = new PhotonPoseEstimator(m_CompetitionAprilTagFieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.kRobotToRightCam);
    m_rightCamPhotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    m_backcamPub = NetworkTableInstance.getDefault()
        .getStructTopic("BackCamPose", Pose2d.struct).publish();
    m_frontcamPub = NetworkTableInstance.getDefault()
        .getStructTopic("FrontCamPose", Pose2d.struct).publish();
    m_rightcamPub = NetworkTableInstance.getDefault()
        .getStructTopic("RightCamPose", Pose2d.struct).publish();

    DataLog log = DataLogManager.getLog();
    m_rightLog = new IntegerLogEntry(log, "RightCamTargets");
    m_frontLog = new IntegerLogEntry(log, "FrontCamTargets");
    m_backLog = new IntegerLogEntry(log, "BackCamTargets");
    m_targetIds = new IntegerLogEntry(log, "TargetIDs");

    m_visionFilter = new VisionFilteringTagDistanceStrategy(m_CompetitionAprilTagFieldLayout);
    initialize();
  }
    
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    logVisionTargets();
  }

  public void initialize() {
     ShuffleboardTab vTab = Shuffleboard.getTab("Vision");

     nt_visionEnabled = vTab.add("Enable vision ", true)
     .withWidget(BuiltInWidgets.kToggleSwitch)
     .withSize(1,1).getEntry();

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
      result = m_frontCamera.getLatestResult();
      targets = result.getTargets();
      if (result.hasTargets()) {
        m_frontLog.append(targets.size());
        targets.forEach(id -> m_targetIds.append(id.getFiducialId()));
      }
      else {
        m_frontLog.append(0);
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
  private Optional<EstimatedRobotPose> getFCEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
      m_frontCamPhotonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
      PhotonPipelineResult result = m_frontCamera.getLatestResult();
      return m_frontCamPhotonPoseEstimator.update(result);
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

      var pose = getFCEstimatedGlobalPose(poseEstimator.getEstimatedPosition());
      if (pose.isPresent()) {
        var pose2d = pose.get().estimatedPose.toPose2d();
        m_frontcamPub.set(pose2d);
        poseEstimator.addVisionMeasurement(pose2d, pose.get().timestampSeconds);
        received_vision_update = true;
 //       System.out.println(" Updated pose with left cam vision.  x:" + pose2d.getX() + "   y: " + pose2d.getY());
      }
      pose = getRCEstimatedGlobalPose(poseEstimator.getEstimatedPosition());
      if (pose.isPresent()) {
        var pose2d = pose.get().estimatedPose.toPose2d();
        m_rightcamPub.set(pose2d);
        poseEstimator.addVisionMeasurement(pose2d, pose.get().timestampSeconds);
 //       System.out.println(" Updated pose with right cam vision.  x:" + pose2d.getX() + "   y: " + pose2d.getY());
        received_vision_update = true;
      }
      pose = getBCEstimatedGlobalPose(poseEstimator.getEstimatedPosition());
      if (pose.isPresent()) {
        var pose2d = pose.get().estimatedPose.toPose2d();
        m_backcamPub.set(pose2d);
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
    //builder.addBooleanProperty("Vision Location Enabled", this::getVisionEnable, this::enableVisionPose);
  }

  public boolean rightCamHasTarget(){
    return m_rightCamera.getLatestResult().hasTargets();
  }

  public boolean backCamHasTarget() {
    return m_backCamera.getLatestResult().hasTargets();
  }

  public boolean frontCamHasTarget() {
    return m_frontCamera.getLatestResult().hasTargets();
  }

  public boolean rightCamConnected() {
    return m_rightCamera.isConnected();
  }

  public boolean backCamConnected() {
    return m_backCamera.isConnected();
  }

  public boolean frontCamConnected() {
    return m_frontCamera.isConnected();
  }
  
  public boolean getVisionEnable(){
    return nt_visionEnabled.getBoolean(true);
  }
  public boolean enableVisionPose(boolean b) {
    m_visionEnabled = b;
    return m_visionEnabled;
  }

}
