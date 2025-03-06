// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.RangeSpeedLimiter;


@Logged
public class ElevatorSubsystem extends SubsystemBase {

  @NotLogged
  ArmSubsystem armSubsystem;
  
  private final SparkFlexConfig config = new SparkFlexConfig();
  private final LimitSwitchConfig limitConfig = new LimitSwitchConfig();
  private final SparkFlex m_elevator = new SparkFlex(Constants.ElevatorConstants.kElevatorCanbusID, MotorType.kBrushless);
  private final RelativeEncoder m_encoder = m_elevator.getEncoder();
  private boolean encoderCalibrated = false;

  private GenericEntry elevator_speed_entry;
  private GenericEntry elevator_height_entry; // only used while tuning fixed heights

  private RangeSpeedLimiter rangespeed;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem(ArmSubsystem _ArmSubsystem) {

    armSubsystem = _ArmSubsystem;
    rangespeed = new RangeSpeedLimiter(250, 0, 20, true, m_elevator, ()-> getHeight());

    config.apply(limitConfig);
 //   initialize();

    //shuffleboard setup w/dashboard editing
    ShuffleboardTab elevatorTab = Shuffleboard.getTab("Elevator");
  
    elevator_speed_entry = elevatorTab
        .addPersistent("Elevator Speed", 0.25)
        .withSize(3, 1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1))
        .getEntry();

    elevator_height_entry = elevatorTab
        .addPersistent("Elevator height", 0.0)
        .withSize(3, 1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 250))
        .getEntry();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    rangespeed.profileEndMotion();
  }
  
    private double m_elevatorDesiredHeight = 0.0; // in arbitrary elevator encoder units

    public Command elevateLevelOne(){
      return cmdElevatorToHeight(Constants.ElevatorConstants.kElevatorHeightL1);  
    }
    public Command elevateLevelTwo(){
      return cmdElevatorToHeight(Constants.ElevatorConstants.kElevatorHeightL2);  
    }
    public Command elevateLevelThree(){
      return cmdElevatorToHeight(Constants.ElevatorConstants.kElevatorHeightL3);  
    }
    public Command elevateLevelFour(){
      return cmdElevatorToHeight(Constants.ElevatorConstants.kElevatorHeightL4);  
    }
  
    public double getSpeed() {
      return elevator_speed_entry.getDouble(0.25);
    }

    public boolean isCalibrated() {
      return encoderCalibrated;
    }

    public double getHeight() {
      return -m_encoder.getPosition();
    }

    public void ManualElevatorUp(){
      m_elevator.set(-1 * getSpeed());
    }
  
    public void ManualElevatorDown(){
      m_elevator.set(1 * getSpeed());
    }
  
    public Command elevatorUp(){
      return this.startEnd(
        () -> m_elevator.set(-1 * getSpeed()),
        () -> m_elevator.stopMotor()
      );
  
    }
  
    public Command elevatorDown(){
      return this.startEnd(
        () -> m_elevator.set(1 * getSpeed()),
        () -> m_elevator.stopMotor()
      );
    }
  
    public Command cmdElevatorToHeight(double height) {
      return this.startRun(
        ()->setDesiredHeight(height),
        ()->moveElevatorToDesiredHeight()).until(() -> elevatorAtDesiredHeight());
    } 

    /*
     * this command is for tuning the desired elevator heights - 
     * it will seek to the height pulled from the shuffleboard widget
     */
    public Command cmdElevatorToShuffleboardHeight() {
      return cmdElevatorToHeight( elevator_height_entry.getDouble(0.0));
    }

    

    public boolean atBottomLimit(){
      return m_elevator.getForwardLimitSwitch().isPressed();
    }
    public boolean atTopLimit(){
      return m_elevator.getReverseLimitSwitch().isPressed();
    }

    private void setDesiredHeight(double desiredHeight) {
      m_elevatorDesiredHeight = desiredHeight;
    }

    /*
     * either moves the elevator to the desired height, or hold it there if at the goal
     */
    public void moveElevatorToDesiredHeight() {

      if (elevatorAtDesiredHeight()) {
        // we've reached the goal, hold now
        stopElevator();
      } else if (getHeight() < m_elevatorDesiredHeight) {
        // need to move up to desired height
        m_elevator.set(-1 * getSpeed());
      } else if (getHeight() > m_elevatorDesiredHeight) {
        // move down to desired height
        m_elevator.set(getSpeed());
      }
    }

    protected boolean elevatorAtDesiredHeight() {
      // see if we're "close enough" to the target height
      if (Math.abs(getHeight() - m_elevatorDesiredHeight) <= Constants.ElevatorConstants.kElevatorHeightTolerance) {
        // close enough
        return true;
      }
      return false;
    }


  public Command elevatorCalibrate() {
    Command c = new ConditionalCommand(
      Commands.none(),//Commands.idle(),  //already calibrated
     
      new SequentialCommandGroup( //needs calibration
      //if not at bottom limit, go there, and make sure the arm is out of the way.  
      new ConditionalCommand(
          Commands.none(), //no arm or elevator motion needed
          new SequentialCommandGroup(
            armSubsystem.cmdArmPositionThatFinishes(20),
            new InstantCommand(() -> m_elevator.set(0.2)) //slow down
          ),
          () -> atBottomLimit()
        ),
        //now wait till bottom limit is hit, and set right values.
        new WaitUntilCommand(() -> atBottomLimit()),
        new InstantCommand(() -> m_elevator.stopMotor()), //stop
        new InstantCommand(() -> m_encoder.setPosition(0.0)),
        new InstantCommand(() -> encoderCalibrated = true)//,
        //Commands.idle()
      ), 
    () -> encoderCalibrated);
    c.addRequirements(this, armSubsystem);
    c.setName("ElevatorCalCommand");
    return c;
  }


  public void stopElevator(){
    m_elevator.set(0);
  }

}
