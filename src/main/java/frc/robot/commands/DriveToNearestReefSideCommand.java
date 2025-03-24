package frc.robot.commands;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Courtesy of team 7414.   See https://www.chiefdelphi.com/t/alignment-pathfinder/492932/15

import java.util.HashMap;
import java.util.List;

import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.Map;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.AprilTagPositions;
//import frc.robot.util.AprilTagPositionsFromLayout;
import frc.robot.subsystems.SwerveSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToNearestReefSideCommand extends Command {
  private Command fullPath;
  private PathPlannerPath m_pathtofront;
  private SwerveSubsystem drive;
  private boolean isLeftSide = false;
  public static final Time kAlignmentAdjustmentTimeout = Seconds.of(0.075);


  /** Creates a new DriveToNearestReefSideCommand. */
  public DriveToNearestReefSideCommand(SwerveSubsystem drive, boolean isLeftSide) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.isLeftSide = isLeftSide;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d closestAprilTagPose = getClosestReefAprilTagPose();
    ArrayList<Pose2d> poses = new ArrayList<>();
    poses.add(closestAprilTagPose);
    drive.m_field.getObject("translatedtag").setPoses(poses);
    // find a path from wherever we are to the stand-off from the closest tag
    
    Pose2d standoffAprilTagPose = translateCoord(closestAprilTagPose, closestAprilTagPose.getRotation().getDegrees(), -1.5);
    
    Pose2d rotatedStandoffAprilTagPose = new Pose2d(standoffAprilTagPose.getX(), standoffAprilTagPose.getY(),
     standoffAprilTagPose.getRotation().rotateBy(Rotation2d.k180deg));


    Command pathfindPath = AutoBuilder.pathfindToPose(
      rotatedStandoffAprilTagPose,
        new PathConstraints(
            3.0, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720)));

    try {
      // Path from the standoff point, to the final position (left or right of tag, as selected)
      Rotation2d backinRot = closestAprilTagPose.getRotation().rotateBy(Rotation2d.k180deg);
      GoalEndState goal = new GoalEndState(0.0, backinRot);
      Pose2d endPose = new Pose2d(closestAprilTagPose.getX(), closestAprilTagPose.getY(), backinRot);
      m_pathtofront = new PathPlannerPath(
          PathPlannerPath.waypointsFromPoses(
            rotatedStandoffAprilTagPose,
            endPose),
          new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI),
          null, goal);
      m_pathtofront.preventFlipping = true;
      fullPath = pathfindPath.andThen(AutoBuilder.followPath(m_pathtofront));

       pathfindPath = AutoBuilder.pathfindToPose(
      endPose,
        new PathConstraints(
            3.0, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720)));
 pathfindPath.schedule();
// if simple findpath to the tag isn't close enough, then do:
//fullPath = pathfindPath.andThen( FinalAlignCommand.generateCommand(drive, endPose, kAlignmentAdjustmentTimeout));
//fullPath.schedule();
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
    }
  }

  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (fullPath != null) {
      fullPath.cancel();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private Pose2d getClosestReefAprilTagPose() {
    HashMap<Integer, Pose2d> aprilTagsToAlignTo = AprilTagPositions.WELDED_BLUE_CORAL_APRIL_TAG_POSITIONS;
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Red) {
        aprilTagsToAlignTo = AprilTagPositions.WELDED_RED_CORAL_APRIL_TAG_POSITIONS;
      }
    }

    Pose2d currentPose = drive.getPose();
    Pose2d closestPose = new Pose2d();
    double closestDistance = Double.MAX_VALUE;
    Integer aprilTagNum = -1;

    for (Map.Entry<Integer, Pose2d> entry : aprilTagsToAlignTo.entrySet()) {
      Pose2d pose = entry.getValue();
      double distance = findDistanceBetween(currentPose, pose);
      if (distance < closestDistance) {
        closestDistance = distance;
        closestPose = pose;
        aprilTagNum = entry.getKey();
        
      }


    }
    ArrayList<Pose2d> poses = new ArrayList<>();
    poses.add(closestPose);
    drive.m_field.getObject("nativetag").setPoses(poses);
  
    System.out.println(" Selected April tag: "+aprilTagNum);
    Pose2d inFrontOfAprilTag = translateCoord(closestPose, closestPose.getRotation().getDegrees(),
        -Units.inchesToMeters(23.773));

    Pose2d leftOrRightOfAprilTag;
    if (isLeftSide) {  // translate distance from tag center to pole center
      leftOrRightOfAprilTag = translateCoord(inFrontOfAprilTag, closestPose.getRotation().getDegrees() + 90, 0.1432265);
    } else {
      leftOrRightOfAprilTag = translateCoord(inFrontOfAprilTag, closestPose.getRotation().getDegrees() + 90, -0.1432265);
    }

    if (List.of(11, 10, 9, 22, 21, 20).contains(aprilTagNum)) {
      if (isLeftSide) {
        leftOrRightOfAprilTag = translateCoord(inFrontOfAprilTag, closestPose.getRotation().getDegrees() + 90, -0.1432265);
      } else {
        leftOrRightOfAprilTag = translateCoord(inFrontOfAprilTag, closestPose.getRotation().getDegrees() + 90, 0.1432265);
      }
    }

    return leftOrRightOfAprilTag;
  }

  private Pose2d translateCoord(Pose2d originalPose, double degreesRotate, double distance) {
    double newXCoord = originalPose.getX() + (Math.cos(Math.toRadians(degreesRotate)) * distance);
    double newYCoord = originalPose.getY() + (Math.sin(Math.toRadians(degreesRotate)) * distance);

    return new Pose2d(newXCoord, newYCoord, originalPose.getRotation());
  }

  private double findDistanceBetween(Pose2d pose1, Pose2d pose2) {
    return Math.sqrt(Math.pow((pose2.getX() - pose1.getX()), 2) + Math.pow((pose2.getY() - pose1.getY()), 2));
  }
}
