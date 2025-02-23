// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ChaseTagCommand;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionPoseEstimationSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
//import com.pathplanner.lib.util.PIDConstants;
//import com.pathplanner.lib.util.ReplanningConfig;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@Logged
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private VisionPoseEstimationSubsystem m_visionPoseEstimationSubsystem = new VisionPoseEstimationSubsystem();
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(m_visionPoseEstimationSubsystem);
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();

  private final PowerDistribution pdp = new PowerDistribution(0,ModuleType.kCTRE);

  private SwerveJoystickCmd swerveJoystickCmd;
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driveXboxController = 
    new CommandXboxController(0);
//  private final Joystick m_buttonBoard = new Joystick(1);
//  private final Trigger m_button1 = new Trigger(() ->m_buttonBoard.getRawButton(1));
  //private final CommandXboxController m_driverController =
  //    new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandJoystick m_reefButtons = new CommandJoystick(1);
  private final CommandJoystick m_outerButtons = new CommandJoystick(2);

  private final SendableChooser<Command> autoChooser;
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    UsbCamera riocam_intake = CameraServer.startAutomaticCapture();
    riocam_intake.setFPS(5);
    riocam_intake.setResolution(160, 120);

    UsbCamera riocam_shooter = CameraServer.startAutomaticCapture();
    riocam_shooter.setFPS(5);
    riocam_shooter.setResolution(160, 120);
    
    // Configure the trigger bindings

    swerveJoystickCmd = new SwerveJoystickCmd(
      swerveSubsystem,
      m_driveXboxController);
    swerveSubsystem.setDefaultCommand(swerveJoystickCmd); 

    //Named commands for Autos



    

    // make the chasetag command

    Command placeholderChaser = new ChaseTagCommand(m_visionSubsystem, swerveSubsystem);
    
    configureBindings();

    //   for debugging only - make a default chase command
   // CommandScheduler.getInstance().setDefaultCommand(m_visionSubsystem, placeholderChaser);

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    setupShuffleboard();
    
    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData("poseestimator", m_visionPoseEstimationSubsystem);
    SmartDashboard.putData("Power", pdp);
    SmartDashboard.putData("Arm", armSubsystem);

  }

  private void setupShuffleboard() {
    
    ShuffleboardTab sys = Shuffleboard.getTab("Systems");

    //sys.add(pdp);
    

  }


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`


    m_driveXboxController.button(4).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));

    m_driveXboxController.button(2).onTrue(new InstantCommand(()->elevatorSubsystem.elevatorUp()));
    m_driveXboxController.button(1).onTrue(new InstantCommand(()->elevatorSubsystem.elevatorDown()));
    m_driveXboxController.button(3).onTrue(new InstantCommand(()->elevatorSubsystem.stopElevator()));
    
    //Command navToA = makeNavCommand(new Pose2d(1.81, 7.68, new Rotation2d(0)));
    //m_driverController.a().whileTrue(navToA);

    //m_driverController.x().whileTrue(new ChaseTagCommand(m_visionSubsystem, swerveSubsystem, m_led));

    // Left Bumper controls field orientation for drive mode. Upressed (default) is field oriented
    //     Pressed is robot oriented
    m_driveXboxController.button(5)
      .onTrue(new InstantCommand(() -> swerveJoystickCmd.setFieldOriented(false)))
      .onFalse(new InstantCommand(() -> swerveJoystickCmd.setFieldOriented(true)));

      //reverse robot orientation mode
      m_driveXboxController.axisGreaterThan(3, 0.5).and(m_driveXboxController.button(5).negate())
      .onTrue(new InstantCommand(() -> swerveJoystickCmd.setReverseFieldOriented(true)))
      .onFalse(new InstantCommand(() -> swerveJoystickCmd.setReverseFieldOriented(false)));

    /*
    note: there is an odd state you can get into with the dampen and boost features enabled below. As the press and
          release are different operations, you can cause odd behaviors. Consider the following sequence:
        Press bumper(set dampened)
        Press Trigger(set Turbo)
        Release Trigger (Set Normal)
        now, the speed is normal, in spite of still holding the bumper in.
        this can be resolved by considering the state of both buttons when choosing the speed factor.

        Fix below may work (also applied above) that allows consideration of other button states when applying the commands.
     */

    m_driveXboxController.button(6)
      .onTrue(new InstantCommand(() -> swerveJoystickCmd.setMotionScale(swerveSubsystem.getDampenedSpeedFactor())))
      .onFalse(new InstantCommand(() -> swerveJoystickCmd.setMotionScale(swerveSubsystem.getNormalSpeedFactor())));

    //includes logic to not activate if dampen is pressed.
      m_driveXboxController.axisGreaterThan(4, 0.5).and(m_driveXboxController.button(6).negate())
      .onTrue(new InstantCommand(() -> swerveJoystickCmd.setMotionScale(swerveSubsystem.getTurboSpeedFactor())))
      .onFalse(new InstantCommand(() -> swerveJoystickCmd.setMotionScale(swerveSubsystem.getNormalSpeedFactor())));

      m_driveXboxController.povUp().onTrue(new InstantCommand(()-> armSubsystem.elbowUp()));
      m_driveXboxController.povDown().onTrue(new InstantCommand(()-> armSubsystem.elbowDown()));
      m_driveXboxController.povLeft().onTrue(new InstantCommand(()-> armSubsystem.stopElbow()));
      m_driveXboxController.povRight().onTrue(new InstantCommand(()-> armSubsystem.setArmHorizontal()));

      m_driveXboxController.button(7).onTrue(new InstantCommand(()-> armSubsystem.wristLeft()));
      m_driveXboxController.button(8).onTrue(new InstantCommand(()-> armSubsystem.wristRight()));
      m_driveXboxController.povRight().onTrue(new InstantCommand(()-> armSubsystem.stopWrist()));
//REMEMBER: YOU NEED AT LEAST 3 USB PORTS TO RUN THIS BUILD!
/* 
    m_reefButtons.button(Constants.ButtonboardConstants.kReefRedLbuttonID).onTrue(new InstantCommand(()-> System.out.println("Button " + 1 + " on Reef Buttons pressed")));
    m_reefButtons.button(Constants.ButtonboardConstants.kReefRedRbuttonID).onTrue(new InstantCommand(()-> System.out.println("Button " + 2 + " on Reef Buttons pressed")));
    m_reefButtons.button(Constants.ButtonboardConstants.kReefGreenTbuttonID).onTrue(new InstantCommand(()-> System.out.println("Button " + 3 + " on Reef Buttons pressed")));
    m_reefButtons.button(Constants.ButtonboardConstants.kReefGreenBbuttonID).onTrue(new InstantCommand(()-> System.out.println("Button " + 4 + " on Reef Buttons pressed")));
    m_reefButtons.button(Constants.ButtonboardConstants.kReefWhiteTbuttonID).onTrue(new InstantCommand(()-> System.out.println("Button " + 5 + " on Reef Buttons pressed")));
    m_reefButtons.button(Constants.ButtonboardConstants.kReefWhiteBbuttonID).onTrue(new InstantCommand(()-> System.out.println("Button " + 6 + " on Reef Buttons pressed")));
    m_reefButtons.button(Constants.ButtonboardConstants.kReefBlueRbuttonID).onTrue(new InstantCommand(()-> System.out.println("Button " + 7 + " on Reef Buttons pressed")));
    m_reefButtons.button(Constants.ButtonboardConstants.kReefBlueLbuttonID).onTrue(new InstantCommand(()-> System.out.println("Button " + 8 + " on Reef Buttons pressed")));
    m_reefButtons.button(Constants.ButtonboardConstants.kReefYellowBbuttonID).onTrue(new InstantCommand(()-> System.out.println("Button " + 9 + " on Reef Buttons pressed")));
    m_reefButtons.button(Constants.ButtonboardConstants.kReefYellowTbuttonID).onTrue(new InstantCommand(()-> System.out.println("Button " + 10 + " on Reef Buttons pressed")));
    m_reefButtons.button(Constants.ButtonboardConstants.kReefPersonbuttonID).onTrue(new InstantCommand(()-> System.out.println("Button " + 11 + " on Reef Buttons pressed")));
    m_reefButtons.button(Constants.ButtonboardConstants.kReefCoinbuttonID).onTrue(new InstantCommand(()-> System.out.println("Button " + 12 + " on Reef Buttons pressed")));
     
    m_outerButtons.button(Constants.ButtonboardConstants.kOuterMaxbuttonID).onTrue(new InstantCommand(()-> System.out.println("Button " + 1 + " on Outer Buttons pressed")));
    m_outerButtons.button(Constants.ButtonboardConstants.kOuterUpperMidbuttonID).onTrue(new InstantCommand(()-> System.out.println("Button " + 2 + " on Outer Buttons pressed")));
    m_outerButtons.button(Constants.ButtonboardConstants.kOuterLowerMidbuttonID).onTrue(new InstantCommand(()-> System.out.println("Button " + 3 + " on Outer Buttons pressed")));
    m_outerButtons.button(Constants.ButtonboardConstants.kOuterMinbuttonID).onTrue(new InstantCommand(()-> System.out.println("Button " + 4 + " on Outer Buttons pressed")));
    m_outerButtons.button(Constants.ButtonboardConstants.kOuterLLIntakebuttonID).onTrue(new InstantCommand(()-> System.out.println("Button " + 5 + " on Outer Buttons pressed")));
    m_outerButtons.button(Constants.ButtonboardConstants.kOuterLRIntakebuttonID).onTrue(new InstantCommand(()-> System.out.println("Button " + 6 + " on Outer Buttons pressed")));
    m_outerButtons.button(Constants.ButtonboardConstants.kOuterRLIntakebuttonID).onTrue(new InstantCommand(()-> System.out.println("Button " + 7 + " on Outer Buttons pressed")));
    m_outerButtons.button(Constants.ButtonboardConstants.kOuterRRIntakebuttonID).onTrue(new InstantCommand(()-> System.out.println("Button " + 8 + " on Outer Buttons pressed")));
    m_outerButtons.button(Constants.ButtonboardConstants.kOuterProcessorbuttonID).onTrue(new InstantCommand(()-> System.out.println("Button " + 9 + " on Outer Buttons pressed")));
    m_outerButtons.button(Constants.ButtonboardConstants.kOuterBargebuttonID).onTrue(new InstantCommand(()-> System.out.println("Button " + 10 + " on Outer Buttons pressed"))); 
    */
  }

    public void runStartupCalibration(){
    /*if(!armSubsystem.isCalibrated()){
      new CalibrateArmCommand(armSubsystem).schedule();
    }*/
  }

  /**
   * @param targetPose
   * @return
   */
  public Command makeNavCommand(Pose2d targetPose){

    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
            0.5, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));

    // See the "Follow a single path" example for more info on what gets passed here
    Command pathfindingCommand = AutoBuilder.pathfindToPose(
        targetPose,
        constraints,
        0.0 // Goal end velocity in meters/sec
    );

    return pathfindingCommand;
  }

  public Command makePathCommand(String pathName) {
    PathPlannerPath path = null;
    // Load the path you want to follow using its name in the GUI
    try {
      path = PathPlannerPath.fromPathFile(pathName);
    } catch (Exception e) {
      DriverStation.reportError("Failed to load PathPlanner file:" + pathName, e.getStackTrace());
    }

    // Create a path following command using AutoBuilder. This will also trigger
    // event markers.
    return AutoBuilder.followPath(path);
  };

  public void loadPreferences(){
    swerveSubsystem.loadPreferences();
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
    //return new NothingCommand();
  }
}
