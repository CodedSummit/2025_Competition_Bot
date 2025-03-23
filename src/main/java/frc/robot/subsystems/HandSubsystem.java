// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

@Logged
public class HandSubsystem extends SubsystemBase {

  private final SparkMax m_hand = new SparkMax(7, MotorType.kBrushless);
  public final SparkLimitSwitch handLimit = m_hand.getReverseLimitSwitch();
  private DigitalInput algeaLimitSwitch = new DigitalInput(6);
  private DigitalInput coralBeamBreak = new DigitalInput(7);
  private GenericEntry nt_coralIntakeSpeed;
  private GenericEntry nt_coralReleaseSpeed;
  private GenericEntry nt_algeaIntakeSpeed;
  private GenericEntry nt_algeaReleaseSpeed;

  /** Creates a new ExampleSubsystem. */
  public HandSubsystem() {
    setupShuffleboard();
  }

  @Logged
  public boolean hasAlgea(){
    return !algeaLimitSwitch.get();
  }
  
  public boolean hasCoral(){
    return !coralBeamBreak.get();
  }

  public boolean hasPiece(){
    if(hasCoral() || hasAlgea()){
      return true;
    }
    return false;
  }


  public void setHandSpeed(double speed){
    m_hand.set(speed);
  }

  public void stopIntake(){
    m_hand.set(0);
  }

  public Command manualIntakeCoral(){
    return this.startEnd(
      ()-> setHandSpeed(getCoralIntakeSpeed()), 
      ()-> setHandSpeed(0));
  }

  public Command manualReleaseCoral(){
    return this.startEnd(
      ()-> setHandSpeed(getCoralReleaseSpeed()), 
      ()-> setHandSpeed(0));
  }

  public Command manualIntakeAlgea(){
    return this.startEnd(
      ()-> setHandSpeed(getAlgeaIntakeSpeed()), 
      ()-> setHandSpeed(0));
  }

  public Command manualReleaseAlgea(){
    return this.startEnd(
      ()-> setHandSpeed(getAlgeaReleaseSpeed()), 
      ()-> setHandSpeed(0));
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getCoralIntakeSpeed(){
    double intakeSpeed = nt_coralIntakeSpeed.getDouble(Constants.HandConstants.kCoralIntakeSpeed);
    intakeSpeed *= -1;
    return intakeSpeed;
  }

  public double getCoralReleaseSpeed(){
    double releaseSpeed = nt_coralReleaseSpeed.getDouble(Constants.HandConstants.kCoralReleaseSpeed);
    return releaseSpeed;
  }

  public double getAlgeaIntakeSpeed(){
    double intakeSpeed = nt_algeaIntakeSpeed.getDouble(Constants.HandConstants.kAlgeaIntakeSpeed);
    intakeSpeed *= -1;
    return intakeSpeed;
  }

  public double getAlgeaReleaseSpeed(){
    double releaseSpeed = nt_algeaReleaseSpeed.getDouble(Constants.HandConstants.kAlgeaReleaseSpeed);
    return releaseSpeed;
  }


  private void setupShuffleboard() {

    ShuffleboardTab handTab = Shuffleboard.getTab("Hand");

    nt_coralIntakeSpeed = handTab.addPersistent("Coral Intake Speed", Constants.HandConstants.kCoralIntakeSpeed)
        .withSize(3, 1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1))
        .getEntry();

    nt_coralReleaseSpeed = handTab.addPersistent("Coral Release Speed", Constants.HandConstants.kCoralReleaseSpeed)
        .withSize(3, 1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1))
        .getEntry();

    nt_algeaIntakeSpeed = handTab.addPersistent("Algea Intake Speed", Constants.HandConstants.kAlgeaIntakeSpeed)
        .withSize(3, 1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1))
        .getEntry();

    nt_algeaReleaseSpeed = handTab.addPersistent("Algea Release Speed", Constants.HandConstants.kAlgeaReleaseSpeed)
        .withSize(3, 1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1))
        .getEntry();
  }
}
