package frc.robot.commands;


import static edu.wpi.first.math.util.Units.inchesToMeters;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.AddressableLedSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ChaseTagCommand extends Command {
  
  // Tranforms from the tag associated with the target to the desired robot position.  If the camera is facing forward (robot +X direction), set the 
  // rotation to 180 degrees (so the bot faces the tag - whose +X comes out of the target).  For a rear-facing camera, set the rotation to 0.0 deg.
  // since the bot would already be facing "away" from the target 
  private static final Transform2d AMP_TAG_TO_GOAL = new Transform2d(new Translation2d(inchesToMeters(17.5), 0), Rotation2d.fromDegrees(0.0));
  private static final Transform2d SPEAKER_TAG_TO_GOAL = new Transform2d(new Translation2d(2, 0), Rotation2d.fromDegrees(0.0));
  private static final Transform2d STAGE_TAG_TO_GOAL = new Transform2d(new Translation2d(1, 0), Rotation2d.fromDegrees(0.0));

  private static final Map<FieldGoals, Transform2d> TAG_TO_GOAL_XFORMS;
  static {
    Map<FieldGoals, Transform2d> aMap = new HashMap<>();
    aMap.put(FieldGoals.NONE, AMP_TAG_TO_GOAL);
    aMap.put(FieldGoals.AMP, AMP_TAG_TO_GOAL);
    aMap.put(FieldGoals.SPEAKER, SPEAKER_TAG_TO_GOAL);
    aMap.put(FieldGoals.STAGE, STAGE_TAG_TO_GOAL);
    TAG_TO_GOAL_XFORMS = Collections.unmodifiableMap(aMap);
  }

  public  enum FieldGoals {AMP, SPEAKER, STAGE, NONE};
  private static final Map<FieldGoals,Integer> BLUE_GOALS;
  static {
    Map<FieldGoals,Integer> aMap = new HashMap<>();
    aMap.put(FieldGoals.NONE, 0);
    aMap.put(FieldGoals.AMP, VisionConstants.kBAmpTagID);
    aMap.put(FieldGoals.SPEAKER, VisionConstants.kBSpeakerTagID);
    aMap.put(FieldGoals.STAGE, VisionConstants.kBStageTagID);
    BLUE_GOALS = Collections.unmodifiableMap(aMap);
  }
private static final Map<FieldGoals,Integer> RED_GOALS;
  static {
    Map<FieldGoals,Integer> aMap = new HashMap<>();
    aMap.put(FieldGoals.NONE, 0);
    aMap.put(FieldGoals.AMP, VisionConstants.kRAmpTagID);
    aMap.put(FieldGoals.SPEAKER, VisionConstants.kRSpeakerTagID);
    aMap.put(FieldGoals.STAGE, VisionConstants.kRStageTagID);
    RED_GOALS = Collections.unmodifiableMap(aMap);
  }

   
  private static final double STALE_TARGET_TIME = 2.0;   // how long to wait (sec.) after losing target before giving up
  private static final double DIFF_MAX_THRESHOLD = 0.05;   // Translation difference - in meters
  private static final double DIFF_MIN_RANGE = 0.2;   // Closest we'd get to the target (for scaling threshold)
  private static final double DIFF_MAX_RANGE = 3.0;   // Beyond this range (meters) use the max diff threshold
  private static final double OMEGA_DIFF_THRESHOLD = 1.0;  // Rotation difference - in degrees

  private final VisionSubsystem m_VisionSubsystem;
  private final SwerveSubsystem m_drivetrainSubsystem;
  private final AddressableLedSubsystem m_LedSubsystem;
  

  private Pose2d m_goalPose = new Pose2d();
  private PhotonTrackedTarget m_lastTarget;
  private double m_xRobotPose;
  private double m_yRobotPose;
  private Timer m_TargetLastSeen = null;
  private static int m_tagToChase = 0;
  public static FieldGoals m_chosenGoal = FieldGoals.NONE;
  private Transform2d m_tagToGoalXform = null;
  private static boolean m_boardBuilt= false;
  
  private DriveToPoseCommand m_driveToPoseCmd = null;

  public ChaseTagCommand(
        VisionSubsystem visionSubsystem, 
        SwerveSubsystem drivetrainSubsystem,
        AddressableLedSubsystem LEDSubsystem) {
    this.m_VisionSubsystem = visionSubsystem;
    this.m_drivetrainSubsystem = drivetrainSubsystem;
    
    this.m_LedSubsystem = LEDSubsystem;
   
    addRequirements(visionSubsystem);
    setupShuffleboard();
  }

  @Override
  public void initialize() {
    m_goalPose = new Pose2d();
    m_lastTarget = null;
    
    this.m_driveToPoseCmd = new DriveToPoseCommand(m_goalPose, m_drivetrainSubsystem);
    m_driveToPoseCmd.initialize();
    m_driveToPoseCmd.updateGoal(m_goalPose);
    
    m_TargetLastSeen = new Timer();
    setFieldGoal(m_chosenGoal);

  }
  

  private void setupShuffleboard() {

    if (!m_boardBuilt) {
      ShuffleboardTab vision = Shuffleboard.getTab("Vision");
      vision.add("Set to AMP", new InstantCommand(() -> this.setFieldGoal(FieldGoals.AMP)));
      vision.add("Set to Stage", new InstantCommand(() -> this.setFieldGoal(FieldGoals.STAGE)));
      vision.add("Set to Speaker", new InstantCommand(() -> this.setFieldGoal(FieldGoals.SPEAKER)));
      vision.add("Set to None", new InstantCommand(() -> this.setFieldGoal(FieldGoals.NONE)));
      vision.addInteger("Tag to chase",() -> getTagToChase());
      
      m_boardBuilt = true;
    }
  }

  
  @Override
  public void execute() {
    
      var robotPose = m_drivetrainSubsystem.getPose();
      var target = m_VisionSubsystem.getTargetForTag(m_tagToChase);

      if (target == null && m_lastTarget == null){
        return; //  No target yet, and have not seen one yet.  Come back later.....
      }

      if (target != null){
        //We're still seeing a target
         // restart our timer on fresh target data (start() method is no-op if alredy running)
        m_TargetLastSeen.start();
        m_TargetLastSeen.reset();
        m_LedSubsystem.setEndsYellow();
      }
      
      if (target != null && targetDataSignificantlyDifferent(target, m_lastTarget)) {
        // This is new target data, so recalculate the goal
        m_lastTarget = target;
        System.out.println(" Calculating a new goal");
        // Get the transformation from the camera to the tag (in 2d)
        var camToTarget = target.getBestCameraToTarget();
        var transform = new Transform2d(
            camToTarget.getTranslation().toTranslation2d(),
            camToTarget.getRotation().toRotation2d());

        // Transform the robot's pose to find the tag's pose
        var cameraPose = robotPose.transformBy(VisionConstants.kRobotToBackCam2d);
        Pose2d targetPose = cameraPose.transformBy(transform);

        // Transform the tag's pose to set our goal
        m_goalPose = targetPose.transformBy(m_tagToGoalXform);
        m_driveToPoseCmd.updateGoal(m_goalPose);  
        if (!m_driveToPoseCmd.isScheduled()) m_driveToPoseCmd.schedule();
      }
  }

 
  /**
   *  Returns true if the newTarget is "significantly" different from lastTarget
   */
  private boolean targetDataSignificantlyDifferent(PhotonTrackedTarget newTarget, PhotonTrackedTarget lastTarget) {
    
    if (newTarget == null || lastTarget == null) return true;
    if (newTarget.equals(lastTarget)) return false;   //  exactly alike

    // see if the distances are different enough to care
    if (Math.abs(newTarget.getBestCameraToTarget().getX() - lastTarget.getBestCameraToTarget().getX()) > getDiffThreshold(newTarget)) return true;
    if (Math.abs(newTarget.getBestCameraToTarget().getY() - lastTarget.getBestCameraToTarget().getY()) > getDiffThreshold(newTarget)) return true;
    if (Units.radiansToDegrees(Math.abs(newTarget.getBestCameraToTarget().getRotation().getAngle() - 
          lastTarget.getBestCameraToTarget().getRotation().getAngle())) > OMEGA_DIFF_THRESHOLD) return true;
    
    return false;
    
  }

  /**
   * Calculate a scaled difference threshold.
   * As you get closer, the threshold decreases (allowing more frequent goal updates)
   */
  private double getDiffThreshold(PhotonTrackedTarget tgt){
    double x = tgt.getBestCameraToTarget().getX();
    double y = tgt.getBestCameraToTarget().getY();
    double range = Math.sqrt(x*x + y*y);
    if (range > DIFF_MAX_RANGE) return DIFF_MAX_THRESHOLD;    
    double thresh = (Math.max(range-DIFF_MIN_RANGE,0.01)/DIFF_MAX_RANGE) * DIFF_MAX_THRESHOLD;
    return thresh;
  }

public boolean isFinished(){

    if (m_driveToPoseCmd.isFinished()){
      m_LedSubsystem.setEndsGreen();
      return true;  // we got to where we're going
    } 

    if (m_TargetLastSeen.hasElapsed(STALE_TARGET_TIME)){
      // If we haven't gotten new target data in a while we may have lost sight of it and should stop
      System.out.println("Have not seen the target lately - STOPPING");
      m_LedSubsystem.setEndsRed();
      return true;
    }
    return false;
  }
  @Override
  public void end(boolean interrupted) {
    m_driveToPoseCmd.cancel();
    m_TargetLastSeen.stop();
  }

  public void initSendable(SendableBuilder builder) {
    
    builder.addDoubleProperty("Robot Pose X", () -> m_xRobotPose, null);
    builder.addDoubleProperty("Robot Pose Y", () -> m_yRobotPose, null);
    builder.addDoubleProperty("Goal X", () -> m_goalPose.getX(), null);
    builder.addDoubleProperty("Goal Y", () -> m_goalPose.getY(), null);
    builder.addDoubleProperty("Goal rotation", () -> m_goalPose.getRotation().getDegrees(), null);
    builder.addIntegerProperty("Tag to chase", () -> getTagToChase(), null);
    
  }

  public Integer getTagToChase() {
    return m_tagToChase;
  }

  /**
   * Setup the internal settings for the chosen goal
   * @param goal
   */
  public void setFieldGoal(FieldGoals goal){
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Red){
         m_tagToChase = RED_GOALS.get(goal);
      } else {
         m_tagToChase = BLUE_GOALS.get(goal);
      }
      m_tagToGoalXform = TAG_TO_GOAL_XFORMS.get(goal);
    }
    m_chosenGoal = goal;
    System.out.println("Chose tag:" + goal);
  }

}

