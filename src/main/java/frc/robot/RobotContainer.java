// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.WinchConstants;
import frc.robot.commands.ChaseTagCommand;
import frc.robot.commands.DriveToNearestReefSideCommand;
import frc.robot.commands.NothingCommand;
import frc.robot.commands.SetWheelAlignment;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.SwerveTestA;
import frc.robot.commands.SwerveTestB;
import frc.robot.commands.ZeroOdometry;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FloorIntake;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionPoseEstimationSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.WinchSubsystem;
import frc.robot.subsystems.WristSubsystem;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
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
  public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(m_visionPoseEstimationSubsystem);
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(armSubsystem);
//  private final WristSubsystem wristSubsystem = new WristSubsystem();
  private final HandSubsystem handSubsystem = new HandSubsystem();
//  private final FloorIntake floorIntakeSubsystem = new FloorIntake();
  
  private final PowerDistribution pdp = new PowerDistribution(0,ModuleType.kCTRE);

  private SwerveJoystickCmd swerveJoystickCmd;
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  private final WinchSubsystem winchSubsystem = new WinchSubsystem();

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

  
  private enum Arrangement {
    L1,
    L2,
    L3,
    L4,
    STATION_PICKUP,
    GROUND_PICKUP,
    CLIMB,
    BARGE,
    ALGEA_1,
    ALGEA_2,
    PROCESSOR,
    NONE
  }

  @NotLogged
  private Arrangement selectedAutoArrange = Arrangement.NONE;
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    armSubsystem.setElevatorSystem(elevatorSubsystem);
    
    NamedCommands.registerCommand("L1", ArrangementL1());
    NamedCommands.registerCommand("L2", ArrangementL2());
    NamedCommands.registerCommand("L3", ArrangementL3());
    NamedCommands.registerCommand("L4", ArrangementL4());
    NamedCommands.registerCommand("StationPickup", ArrangementStationPickup());
    NamedCommands.registerCommand("Barge", ArrangementBarge());
    NamedCommands.registerCommand("Intake/Place Coral", smartIntakeCoral());
    NamedCommands.registerCommand("Algea Intake/Place", smartIntakeCoral());
    NamedCommands.registerCommand("Print", new InstantCommand(()-> System.out.println("Autonomous Print Achieved!")));
    NamedCommands.registerCommand("Calibrate", elevatorSubsystem.elevatorCalibrate());
    NamedCommands.registerCommand("Elbow Down", new InstantCommand(()-> armSubsystem.cmdArmPositionThatFinishes(armSubsystem.getArmAngle() - 6)));
    NamedCommands.registerCommand("Calibrate Gyro", swerveSubsystem.zeroHeadingCommand());
    NamedCommands.registerCommand("Left Reef Tag Align", new DriveToNearestReefSideCommand(swerveSubsystem, true));
    NamedCommands.registerCommand("Right Reef Tag Align", new DriveToNearestReefSideCommand(swerveSubsystem, false));
    NamedCommands.registerCommand("Algea 1", ArrangementAlgea1());
    NamedCommands.registerCommand("Algea 2", ArrangementAlgea2());
    
    /*    UsbCamera riocam_intake = CameraServer.startAutomaticCapture();
    riocam_intake.setFPS(5);
    riocam_intake.setResolution(160, 120);
    
    UsbCamera riocam_shooter = CameraServer.startAutomaticCapture();
    riocam_shooter.setFPS(5);
    riocam_shooter.setResolution(160, 120);
    */  
    // Configure the trigger bindings
    
    swerveJoystickCmd = new SwerveJoystickCmd(
      swerveSubsystem,
      m_driveXboxController);
      swerveSubsystem.setDefaultCommand(swerveJoystickCmd); 

      // make the chasetag command
      
      Command placeholderChaser = new ChaseTagCommand(m_visionSubsystem, swerveSubsystem);
      
      configureBindings();
      
      //   for debugging only - make a default chase command
      // CommandScheduler.getInstance().setDefaultCommand(m_visionSubsystem, placeholderChaser);
      
      //Named Commands for PathPlanner



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

  public boolean isRedAlliance(){
    return DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red);
  }

  private void setupShuffleboard() {
    
    ShuffleboardTab tab = Shuffleboard.getTab("Systems");

    //sys.add(pdp);
//    tab.add(wristSubsystem);
    tab.add(armSubsystem);
    

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
    m_driveXboxController.y().whileTrue(armSubsystem.manualElbowUp());
    m_driveXboxController.a().whileTrue(armSubsystem.manualElbowDown());
    m_driveXboxController.x().onTrue(swerveSubsystem.zeroHeadingCommand());
    m_driveXboxController.b().onTrue(smartIntakeAlgea());

    //m_driveXboxController.b().whileTrue(floorIntakeSubsystem.Intake());
    
    //    m_outerButtons.button(Constants.ButtonboardConstants.kOuterProcessorbuttonID).whileTrue(floorIntakeSubsystem.Intake());
    //    m_outerButtons.button(Constants.ButtonboardConstants.kOuterBargebuttonID).whileTrue(floorIntakeSubsystem.Outtake());
    
    
    
    m_driveXboxController.leftBumper().whileTrue(elevatorSubsystem.elevatorUp());
    m_driveXboxController.leftTrigger().whileTrue(elevatorSubsystem.elevatorDown());
    
    m_driveXboxController.rightBumper()
    .onTrue(new InstantCommand(() -> swerveJoystickCmd.setMotionScale(swerveSubsystem.getDampenedSpeedFactor())))
    .onFalse(new InstantCommand(() -> swerveJoystickCmd.setMotionScale(swerveSubsystem.getNormalSpeedFactor())));
    
    //includes logic to not activate if dampen is pressed.
    m_driveXboxController.rightTrigger(.5).and(m_driveXboxController.rightBumper().negate())
    .onTrue(new InstantCommand(() -> swerveJoystickCmd.setMotionScale(swerveSubsystem.getTurboSpeedFactor())))
    .onFalse(new InstantCommand(() -> swerveJoystickCmd.setMotionScale(swerveSubsystem.getNormalSpeedFactor())));
    //m_driveXboxController.povUp().onTrue(swerveSubsystem.zeroHeadingCommand());
    
    m_driveXboxController.button(7).onTrue(smartIntakeCoral());
    m_driveXboxController.button(8).onTrue(AutoArrangeCommand);

    m_driveXboxController.button(9).whileTrue(armSubsystem.manualElbowUp());
    m_driveXboxController.button(10).whileTrue(armSubsystem.manualElbowDown());
    
//    m_driveXboxController.povUp().whileTrue(floorIntakeSubsystem.ManualArmIn());
//    m_driveXboxController.povDown().whileTrue(floorIntakeSubsystem.ManualArmOut());

    m_driveXboxController.povRight().whileTrue(handSubsystem.manualIntakeAlgea());
    m_driveXboxController.povLeft().whileTrue(handSubsystem.manualReleaseAlgea());  
    
//;    m_driveXboxController.povRight().onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeadingWithVision(isRedAlliance())));

    m_driveXboxController.povDown().whileTrue(winchSubsystem.retractWinch());
    m_driveXboxController.povUp().whileTrue(winchSubsystem.extendWinch());
    //testing
    //m_driveXboxController.povLeft().whileTrue(floorIntakeSubsystem.moveArmToPosition(FloorIntake.UP_POSITION));
    //m_driveXboxController.povRight().whileTrue(floorIntakeSubsystem.moveArmToPosition(FloorIntake.ALGEA_POSITION));
    
    //real code
//    m_driveXboxController.povLeft().whileTrue(wristSubsystem.manualWristLeft());
 //   m_driveXboxController.povRight().whileTrue(wristSubsystem.manualWristRight());
    
    // Left Bumper controls field orientation for drive mode. Upressed (default) is field oriented
    //     Pressed is robot oriented
    /* 
    m_driveXboxController.button(5)
      .onTrue(new InstantCommand(() -> swerveJoystickCmd.setFieldOriented(false)))
      .onFalse(new InstantComma nmnd(() -> swerveJoystickCmd.setFieldOriented(true)));*/
/* 
      //reverse robot orientation mode
      m_driveXboxController.axisGreaterThan(3, 0.5).and(m_driveXboxController.button(5).negate())
      .onTrue(new InstantCommand(() -> swerveJoystickCmd.setReverseFieldOriented(true)))
      .onFalse(new InstantCommand(() -> swerveJoystickCmd.setReverseFieldOriented(false)));
*/

 //     m_driveXboxController.povDown().onTrue(armSubsystem.cmdArmHorizontal());
 //m_driveXboxController.povUp().onTrue(PositionCommand(100, -10, WristSubsystem.CENTER, FloorIntake.ALGEA_POSITION));

      m_outerButtons.button(Constants.ButtonboardConstants.kOuterMaxbuttonID).onTrue(new InstantCommand(()-> setAutoArrangeCommand(Arrangement.L4)));
      m_outerButtons.button(Constants.ButtonboardConstants.kOuterUpperMidbuttonID).onTrue(new InstantCommand(()-> setAutoArrangeCommand(Arrangement.L3)));
      m_outerButtons.button(Constants.ButtonboardConstants.kOuterLowerMidbuttonID).onTrue(new InstantCommand(()-> setAutoArrangeCommand(Arrangement.L2)));
      m_outerButtons.button(Constants.ButtonboardConstants.kOuterMinbuttonID).onTrue(new InstantCommand(()-> setAutoArrangeCommand(Arrangement.L1)));
      m_outerButtons.button(Constants.ButtonboardConstants.kOuterLLIntakebuttonID).onTrue(new InstantCommand(()-> setAutoArrangeCommand(Arrangement.STATION_PICKUP)));
      m_outerButtons.button(Constants.ButtonboardConstants.kOuterLRIntakebuttonID).onTrue(new InstantCommand(()-> setAutoArrangeCommand(Arrangement.CLIMB)));
  

//    m_reefButtons.button(Constants.ButtonboardConstants.kReefRedLbuttonID).onTrue(new SwerveTestA(swerveSubsystem));
//    m_reefButtons.button(Constants.ButtonboardConstants.kReefRedRbuttonID).onTrue(new SwerveTestB(swerveSubsystem));
//  m_reefButtons.button(Constants.ButtonboardConstants.kReefGreenTbuttonID).onTrue(new InstantCommand(()-> System.out.println("Button " + 3 + " on Reef Buttons pressed")));
//  m_reefButtons.button(Constants.ButtonboardConstants.kReefGreenBbuttonID).onTrue(new InstantCommand(()-> System.out.println("Button " + 4 + " on Reef Buttons pressed")));
  m_reefButtons.button(Constants.ButtonboardConstants.kReefWhiteTbuttonID).onTrue(new InstantCommand(()-> setAutoArrangeCommand(Arrangement.ALGEA_2)));
  m_reefButtons.button(Constants.ButtonboardConstants.kReefWhiteBbuttonID).onTrue(new InstantCommand(()-> setAutoArrangeCommand(Arrangement.ALGEA_1)));
//  m_reefButtons.button(Constants.ButtonboardConstants.kReefBlueRbuttonID).onTrue(new InstantCommand(()-> System.out.println("Button " + 7 + " on Reef Buttons pressed")));
//  m_reefButtons.button(Constants.ButtonboardConstants.kReefBlueLbuttonID).onTrue(new InstantCommand(()-> System.out.println("Button " + 8 + " on Reef Buttons pressed")));
  m_reefButtons.button(Constants.ButtonboardConstants.kReefYellowBbuttonID).onTrue(AutoArrangeCommand);
//  m_reefButtons.button(Constants.ButtonboardConstants.kReefYellowTbuttonID).onTrue(handSubsystem.manualIntakeCoral());
//  m_reefButtons.button(Constants.ButtonboardConstants.kReefPersonbuttonID).onTrue(elevatorSubsystem.elevatorCalibrate());
  m_reefButtons.button(Constants.ButtonboardConstants.kReefCoinbuttonID).whileTrue(handSubsystem.manualReleaseAlgea());
     

//  m_outerButtons.button(Constants.ButtonboardConstants.kOuterMaxbuttonID).onTrue(new InstantCommand(()-> System.out.println("Button " + 1 + " on Outer Buttons pressed")));
//  m_outerButtons.button(Constants.ButtonboardConstants.kOuterUpperMidbuttonID).onTrue(new InstantCommand(()-> System.out.println("Button " + 2 + " on Outer Buttons pressed")));
//  m_outerButtons.button(Constants.ButtonboardConstants.kOuterLowerMidbuttonID).onTrue(new InstantCommand(()-> System.out.println("Button " + 3 + " on Outer Buttons pressed")));
//  m_outerButtons.button(Constants.ButtonboardConstants.kOuterMinbuttonID).onTrue(new InstantCommand(()-> System.out.println("Button " + 4 + " on Outer Buttons pressed")));
//  m_outerButtons.button(Constants.ButtonboardConstants.kOuterLLIntakebuttonID).onTrue(new InstantCommand(()-> System.out.println("Button " + 5 + " on Outer Buttons pressed")));
//  m_outerButtons.button(Constants.ButtonboardConstants.kOuterLRIntakebuttonID).onTrue(new InstantCommand(()-> System.out.println("Button " + 6 + " on Outer Buttons pressed")));
    m_outerButtons.button(Constants.ButtonboardConstants.kOuterRLIntakebuttonID).onTrue(new DriveToNearestReefSideCommand(swerveSubsystem, true));
    m_outerButtons.button(Constants.ButtonboardConstants.kOuterRRIntakebuttonID).onTrue(new DriveToNearestReefSideCommand(swerveSubsystem, false));
    m_outerButtons.button(Constants.ButtonboardConstants.kOuterProcessorbuttonID).onTrue(new InstantCommand(()-> setAutoArrangeCommand(Arrangement.PROCESSOR)));
    m_outerButtons.button(Constants.ButtonboardConstants.kOuterBargebuttonID).onTrue(new InstantCommand(()-> setAutoArrangeCommand(Arrangement.BARGE))); 

  }

    public void runStartupCalibration(){
    /*if(!armSubsystem.isCalibrated()){
      new CalibrateArmCommand(armSubsystem).schedule();
    }*/
    elevatorSubsystem.elevatorCalibrate().schedule();
  }

  public Command smartIntakeCoral(){
    return new ConditionalCommand(
      new SequentialCommandGroup( //if has coral
        //elevatorSubsystem.cmdElevatorToHeight(() -> elevatorSubsystem.getHeight() -12),
        //.onlyIf(() ->wristSubsystem.isPieceVertical()),
        new InstantCommand(() -> handSubsystem.setHandSpeed(handSubsystem.getCoralReleaseSpeed())),
        new WaitCommand(1),
        new InstantCommand(() -> handSubsystem.setHandSpeed(0))
      ),
     new SequentialCommandGroup( //if no coral
        new InstantCommand(() -> handSubsystem.setHandSpeed(handSubsystem.getCoralIntakeSpeed())),
        new WaitUntilCommand(() -> handSubsystem.hasCoral()).withTimeout(10),
        new WaitCommand(0),
        new InstantCommand(() -> handSubsystem.setHandSpeed(0))
    ),
    () -> handSubsystem.hasCoral());
  }

  public Command smartIntakeAlgea(){
    return new ConditionalCommand(
      new SequentialCommandGroup( //if has coral
        //elevatorSubsystem.cmdElevatorToHeight(() -> elevatorSubsystem.getHeight() -12),
        //.onlyIf(() ->wristSubsystem.isPieceVertical()),
        new InstantCommand(() -> handSubsystem.setHandSpeed(handSubsystem.getAlgeaReleaseSpeed())),
        new WaitCommand(1),
        new InstantCommand(() -> handSubsystem.setHandSpeed(0))
      ),
     new SequentialCommandGroup( //if no coral
        new InstantCommand(() -> handSubsystem.setHandSpeed(handSubsystem.getAlgeaIntakeSpeed())),
        new WaitUntilCommand(() -> handSubsystem.hasAlgea()).withTimeout(10),
        new WaitCommand(0),
        new InstantCommand(() -> handSubsystem.setHandSpeed(0))
    ),
    () -> handSubsystem.hasAlgea());
  }

  public Command oldPositionCommand(double elevator_position, double arm_angle, double wrist_angle, double floor_position){
    return Commands.parallel( //each of these commands must finish in order to run another PositionCommand.
      elevatorSubsystem.cmdElevatorToHeight(() -> elevator_position),
      armSubsystem.cmdArmPositionThatFinishes(arm_angle)
      //,
    //  wristSubsystem.moveWristToPosition(wrist_angle)
      //,
     // floorIntakeSubsystem.moveArmToPosition(floor_position)
    ).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }

  public Command PositionCommand(double elevator_position, double arm_angle){
    return Commands.parallel( //each of these commands must finish in order to run another PositionCommand.
      elevatorSubsystem.cmdElevatorToHeight(() -> elevator_position).beforeStarting(new WaitUntilCommand(()-> armSubsystem.isSafeForElevator())),
      armSubsystem.cmdArmPositionThatFinishes(arm_angle)
    ).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }

  public Command PositionCommandWinch(double elevator_position, double arm_angle, double winch_position){
    return Commands.parallel( //each of these commands must finish in order to run another PositionCommand.
      elevatorSubsystem.cmdElevatorToHeight(() -> elevator_position).beforeStarting(new WaitUntilCommand(()-> armSubsystem.isSafeForElevator())),
      armSubsystem.cmdArmPositionThatFinishes(arm_angle),
      winchSubsystem.cmdWinchToPositionThatFinishes(winch_position)
    ).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }

  private void setAutoArrangeCommand(Arrangement a){
    System.out.println("setting arrange to " + a);
    selectedAutoArrange = a;
  }
  private Arrangement getAutoArrangeCommand(){
    System.out.println("arrange command: " + selectedAutoArrange);
    return selectedAutoArrange;
  }

    // An example selectcommand.  Will select from the three commands based on the value returned
  // by the selector method at runtime.  Note that selectcommand works on Object(), so the
  // selector does not have to be an enum; it could be any desired type (string, integer,
  // boolean, double...)
  private final Command AutoArrangeCommand =
      new SelectCommand<>(
          // Maps selector values to commands
          Map.ofEntries(
            Map.entry(Arrangement.L4, ArrangementL4()),
            Map.entry(Arrangement.L3, ArrangementL3()),
            Map.entry(Arrangement.L2, ArrangementL2()),
            Map.entry(Arrangement.L1, ArrangementL1()),
            Map.entry(Arrangement.STATION_PICKUP, ArrangementStationPickup()),
              Map.entry(Arrangement.GROUND_PICKUP, new InstantCommand(()-> System.out.println("Ground Pickup!"))),
              Map.entry(Arrangement.BARGE, ArrangementBarge()),
//              Map.entry(Arrangement.CLIMB, ArrangementClimb()),
              Map.entry(Arrangement.PROCESSOR, ArrangementProcessor()),
              Map.entry(Arrangement.ALGEA_1, ArrangementAlgea1()),
              Map.entry(Arrangement.ALGEA_2, ArrangementAlgea2()),
              Map.entry(Arrangement.NONE, new PrintCommand("Arrangement None Requested"))
          ),
          () -> getAutoArrangeCommand());




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
  
  public Command getPathPlannerAuto(){
    return autoChooser.getSelected();
  }

  public void loadPreferences(){
    swerveSubsystem.loadPreferences();
  }
  //IF YOU WANT TO ADD A COMMAND TO PATHPLANNER: Add it here, please. 
  //IMPORTANT! We need to find the values for the elevator and arm before we run these!

  public Command ArrangementL4(){
    return PositionCommand(148, 180);
  }

  public Command ArrangementL3(){
    return PositionCommand(1.5, 180);
  }

  public Command ArrangementL2(){
    return PositionCommand(0, 73);
  }

  public Command ArrangementL1(){
    return PositionCommand(0, 73);
  }

  public Command ArrangementStationPickup(){
    return PositionCommand(21.7, 12.5);
  }

//  public Command ArrangementClimb(){
//    return PositionCommandWinch(0,256.3,182.1);
//  }

  public Command ArrangementBarge(){
    return PositionCommand(150, 180);
  }

  public Command ArrangementAlgea1(){
    return PositionCommand(0, 270);
  }

  public Command ArrangementAlgea2(){
    return PositionCommand(55.8, 270);
  }

  public Command ArrangementProcessor(){
    return PositionCommand(0, 46.6);
  }

  //Commands that are going to Pathplanner should stay above this line.

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
