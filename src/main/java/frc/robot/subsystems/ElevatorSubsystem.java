// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class ElevatorSubsystem extends SubsystemBase {

  private final SparkFlexConfig config = new SparkFlexConfig();
  private final AbsoluteEncoderConfig encoderConfig = new AbsoluteEncoderConfig();
  private final LimitSwitchConfig limitConfig = new LimitSwitchConfig();
  private final SparkFlex m_elevator = new SparkFlex(5, MotorType.kBrushless);
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {

    config.apply(encoderConfig);
    config.apply(limitConfig);
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

  public void elevatorUp(){
    m_elevator.set(1);
  }

  public void elevatorDown(){
    m_elevator.set(-1);
  }

  public void stopElevator(){
    m_elevator.set(0);
  }
}
