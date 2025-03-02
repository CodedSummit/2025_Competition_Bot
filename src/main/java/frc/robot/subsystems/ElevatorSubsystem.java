// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
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

@Logged
public class ElevatorSubsystem extends SubsystemBase {

  @NotLogged
  ArmSubsystem armSubsystem;
  
  private final SparkFlexConfig config = new SparkFlexConfig();
  private final LimitSwitchConfig limitConfig = new LimitSwitchConfig();
  private final SparkFlex m_elevator = new SparkFlex(5, MotorType.kBrushless);
  private final RelativeEncoder m_encoder = m_elevator.getEncoder();
  private boolean encoderCalibrated = false;
  
private GenericEntry elevator_speed_entry;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem(ArmSubsystem _ArmSubsystem) {

    armSubsystem = _ArmSubsystem;

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

    setDefaultCommand(elevatorCalibrate());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ProfileEndMotion();
  }

  // End is the end of motion
  // Start is the start of the motion limiting.
  private double TopEnd = 250;
  private double TopStart = 230;
  private double BottomEnd = 0;
  private double BottomStart = 20;

  private void ProfileEndMotion(){
    //this method is used to automaticaly profile motion as the elevator approaches it's limits.

    //if in upper or lower ranges, then set the max speed based on percentage
    double limit = speedLimitAtCurrentPosition();
    double motor_speed = m_elevator.get();
    if(limit < Math.abs(motor_speed)){
      if(motor_speed > 0){
        m_elevator.set(limit);
      } else {
        m_elevator.set(-limit);
      }
    }

  }

  public double speedLimitAtCurrentPosition(){
    double position = getHeight();
    double motor_direction = m_elevator.get();
    if(TopEnd > position && position > TopStart && motor_direction < 0){ //negative motion is up
      return upperRangePercentage();
    }else if(BottomStart > position && position > BottomEnd && motor_direction > 0){ //positive motion is down.
      return lowerRangePercentage();
    }
    return 1; //no speed limit at current position.

  }

  public double upperRangePercentage(){
    return calculatePercentage(TopEnd, TopStart, getHeight());
  }

  public double lowerRangePercentage(){
    return calculatePercentage(BottomEnd, BottomStart, getHeight());
  }

  public static double calculatePercentage(double start, double end, double current) {
    // Calculate the absolute difference between start and end
    double total = Math.abs(end - start);
    
    // Calculate the difference between current and start
    double progress = Math.abs(current - start);
    
    // Calculate the percentage
    double percentage = (progress / total);
    
    // Ensure the percentage is between 0 and 100
    percentage = Math.min(1, Math.max(0, percentage));
    
    return percentage;
  }

/* 
  public double rangePercentage(double start, double end, double current) {
    if (min == max) {
        throw new IllegalArgumentException("Min and max should not be equal.");
    }
    if(current >= max) return 1;
    if(current <= min) return 0;

    double proportion;
    if (min > max) {
        proportion = (current - max) / (min - max); // Proportion toward the 'minimum' number
    } else {
        proportion = (current - min) / (max - min); // Proportion toward the 'maximum' number
    }

    return proportion ; // Convert to percentage
  }*/

  public void elevateLevelOne(SparkFlex motor){
    System.out.println("Set Elevator to Level 1!");

  }
  public void elevateLevelTwo(SparkFlex motor){
    System.out.println("Set Elevator to Level 2!");
  }
  public void elevateLevelThree(SparkFlex motor){
    System.out.println("Set Elevator to Level 3!");
  }
  public void elevateLevelFour(SparkFlex motor){
    System.out.println("Set Elevator to Level 4!");
  }


  public double getSpeed(){
    return elevator_speed_entry.getDouble(0.25);
  }

public boolean isCalibrated() {
  return encoderCalibrated;
}

  public double getHeight(){
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
      // Start a flywheel spinning at 50% power
      () -> m_elevator.set(-1 * getSpeed()),
      // Stop the flywheel at the end of the command
      () -> m_elevator.stopMotor()
    );

  }

  public Command elevatorDown(){
    return this.startEnd(
      // Start a flywheel spinning at 50% power
      () -> m_elevator.set(1 * getSpeed()),
      // Stop the flywheel at the end of the command
      () -> m_elevator.stopMotor()
    );
  }

  public boolean atBottomLimit(){
    return m_elevator.getForwardLimitSwitch().isPressed();
  }
  public boolean atTopLimit(){
    return m_elevator.getReverseLimitSwitch().isPressed();
  }

  public Command elevatorCalibrate() {
    Command c = new ConditionalCommand(
      Commands.idle(),  //already calibrated
     
      new SequentialCommandGroup( //needs calibration
      //if not at bottom limit, go there, and make sure the arm is out of the way.  
      new ConditionalCommand(
          Commands.none(), //no arm or elevator motion needed
          new SequentialCommandGroup(
            armSubsystem.cmdArmHorizontalThatFinishes(),
            new InstantCommand(() -> m_elevator.set(0.2)) //slow down
          ),
          () -> atBottomLimit()
        ),
        //now wait till bottom limit is hit, and set right values.
        new WaitUntilCommand(() -> atBottomLimit()),
        new InstantCommand(() -> m_elevator.stopMotor()), //stop
        new InstantCommand(() -> m_encoder.setPosition(0.0)),
        new InstantCommand(() -> encoderCalibrated = true),
        Commands.idle()
      ), 
    () -> encoderCalibrated);
    c.addRequirements(this, armSubsystem);
    return c;
  }


  public void stopElevator(){
    m_elevator.set(0);
  }

  protected void initialize(){
    SparkLimitSwitch bottomSwitch = m_elevator.getReverseLimitSwitch();
    while(!bottomSwitch.isPressed()){
      elevatorDown();
    }
    stopElevator();
    m_encoder.setPosition(0.0);
  }
}
