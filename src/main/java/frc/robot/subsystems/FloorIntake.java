// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FloorIntake extends SubsystemBase {
  private final SparkMax m_intakeArm = new SparkMax(9, MotorType.kBrushed);
  private final SparkMax m_intakeHand = new SparkMax(10, MotorType.kBrushed);
  
  /** Creates a new FloorIntake. */
  public FloorIntake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
