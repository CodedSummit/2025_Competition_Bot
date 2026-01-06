// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

@Logged
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private NetworkTableInstance m_oldInst =null;
  private RobotContainer m_robotContainer;
  private String newAutoName, autoName;
  private boolean lastFlipNeeded = false;


  public Robot(){
    Epilogue.configure(config -> {
      config.root = "Telemetry";
    });
    Epilogue.bind(this);
  }
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    DataLogManager.start();
    // Record both DS control and joystick data
    DriverStation.startDataLog(DataLogManager.getLog());
    
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    newAutoName = m_robotContainer.getAutonomousCommand().getName();

    boolean flipNeeded = false;
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent() && ally.get() == Alliance.Red) {
      flipNeeded = true;
    }

    if (autoName != newAutoName || lastFlipNeeded != flipNeeded) {
      autoName = newAutoName;
      lastFlipNeeded = flipNeeded;

      try {
        if (AutoBuilder.getAllAutoNames().contains(autoName)) {
          System.out.println("Displaying " + autoName);
          List<PathPlannerPath> pathPlannerPaths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
          List<Pose2d> poses = new ArrayList<>();
          if (flipNeeded) {
            // flip needed for red side
            for (PathPlannerPath path : pathPlannerPaths) {
              poses.addAll(path.flipPath().getAllPathPoints().stream()
                  .map(point -> new Pose2d(point.position.getX(), point.position.getY(), new Rotation2d()))
                  .collect(Collectors.toList()));
            }
          } else {
            // no flip needed for blue side
            for (PathPlannerPath path : pathPlannerPaths) {
              poses.addAll(path.getAllPathPoints().stream()
                  .map(point -> new Pose2d(point.position.getX(), point.position.getY(), new Rotation2d()))
                  .collect(Collectors.toList()));
            }
          }

          m_robotContainer.swerveSubsystem.m_field.getObject("path").setPoses(poses);
        }
      } catch (Exception e) {
        DriverStation.reportError("Failed to load Auto " + autoName + "'s trajectory", e.getStackTrace());
      }

    }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
//    m_robotContainer.runStartupCalibration();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    m_robotContainer.swerveSubsystem.zeroHeadingWithVision(m_robotContainer.isRedAlliance());
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.loadPreferences();
    m_robotContainer.runStartupCalibration();
    m_robotContainer.swerveSubsystem.zeroHeadingWithVision(m_robotContainer.isRedAlliance());
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    if(isSimulation()) {
      NetworkTableInstance inst = NetworkTableInstance.getDefault();
      if (inst != m_oldInst){
        inst.stopServer();
        // Change the IP address in the below function to the IP address you use to connect to the PhotonVision UI.
        inst.setServer("photonvision.local");
        inst.startClient4("Robot Simulation");
        System.out.println(" Setting new network table for simulation");
        m_oldInst = inst;
      }
    }
  }
}