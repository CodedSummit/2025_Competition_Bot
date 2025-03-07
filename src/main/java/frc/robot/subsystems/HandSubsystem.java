// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalInput;
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

  private final SparkMax m_hand = new SparkMax(8, MotorType.kBrushed);
  public final SparkLimitSwitch handLimit = m_hand.getReverseLimitSwitch();
  private DigitalInput coralLimitSwitch = new DigitalInput(6);


  /** Creates a new ExampleSubsystem. */
  public HandSubsystem() {
    setupShuffleboard();
  }

  @Logged
  public boolean hasCoral(){
    return !coralLimitSwitch.get();
  }

  public void setHandSpeed(double speed){
    m_hand.set(speed);
  }

  public void stopIntake(){
    m_hand.set(0);
  }

  public Command manualIntakeCoral(){
    return this.startEnd(
      ()-> setHandSpeed(-1), 
      ()-> setHandSpeed(0));
  }

  public Command manualReleaseCoral(){
    return this.startEnd(
      ()-> setHandSpeed(1), 
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


  private void setupShuffleboard() {

    ShuffleboardTab tab = Shuffleboard.getTab("Hand");

    /*nt_elbowUPSpeed = armTab.addPersistent("Elbow UP speed", Constants.ArmConstants.kElbowUpSpeed)
        .withSize(3, 1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1))
        .getEntry();

    nt_elbowDOWNSpeed = armTab.addPersistent("Elbow DOWN speed", Constants.ArmConstants.kElbowDownSpeed)
        .withSize(3, 1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1))
        .getEntry();
*/
  }

}
