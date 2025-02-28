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
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class ElevatorSubsystem extends SubsystemBase {

  private final SparkFlexConfig config = new SparkFlexConfig();
  private final LimitSwitchConfig limitConfig = new LimitSwitchConfig();
  private final SparkFlex m_elevator = new SparkFlex(5, MotorType.kBrushless);
  private final RelativeEncoder m_encoder = m_elevator.getEncoder();
  
  
private GenericEntry elevator_speed_entry;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {

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


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

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

  /*public void elevatorUp(){
    m_elevator.set(1);
  }*/

  public double getSpeed(){
    return elevator_speed_entry.getDouble(0.25);
  }

  public double getHeight(){
    return m_encoder.getPosition();
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
