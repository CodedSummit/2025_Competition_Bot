// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

  private SparkMaxConfig config= new SparkMaxConfig();
  private final DigitalInput input = new DigitalInput(7);
  private final DutyCycleEncoder absEncoder = new DutyCycleEncoder(input);
  private final SparkMax m_elbow = new SparkMax(6, MotorType.kBrushless);
  private final SparkMax m_wrist = new SparkMax(7, MotorType.kBrushless);
  private final SparkMax m_intake = new SparkMax(8, MotorType.kBrushless);

 //private final Encoder m_encoder = new Encoder(ArmConstants.kEncoderPorts[0], ArmConstants.kEncoderPorts[1]);
  private final ArmFeedforward m_feedforward = new ArmFeedforward(
      ArmConstants.kSVolts, ArmConstants.kGVolts,
      ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);
  private double m_handlerSpeed = ArmConstants.kHandlerDefaultSpeed;
  private GenericEntry nt_handlerSpeed;

  private ProfiledPIDController elbowPIDController = new ProfiledPIDController(ArmConstants.kP,
  0,
  0,
  new TrapezoidProfile.Constraints(
      ArmConstants.kMaxVelocityRadPerSecond,
      ArmConstants.kMaxAccelerationRadPerSecSquared),
      0);




  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {

    config.apply(config);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //TODO - check that the angle is NOT in the danger zone.   if it is, pull the plug on the motor 
  }

  public void setElbowOutput() {
    // TODO - implement.
    // Calculate the feedforward from the sepoint
    /*
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    double pidOutput = elbowPIDController.calculate(double output, TrapezoidProfile.State setpoint)
    // Add the feedforward to the PID output to get the motor output
    double target_voltage = pidOutput + feedforward;
    
    //limit max voltage at point of applying to motor.
    target_voltage = Math.min(target_voltage, 0.2);

    m_armMotor.setVoltage(target_voltage);
    */
  }


  public void elbowUp(){
    m_elbow.set(-0.1);
  }

  public void elbowDown(){
    m_elbow.set(0.1);
  }

  public void wristLeft(){
    m_wrist.set(0.1);
  }

  public void wristRight(){
    m_wrist.set(-0.1);
  }

  public void intakeCoral(){
    m_intake.set(0.1);
    new WaitCommand(2);
    m_intake.set(0);
  }

  public void placeCoral(){
    m_intake.set(-0.1);
    new WaitCommand(2);
    m_intake.set(0);
  }

  public void stopElbow(){
    m_elbow.set(0);
  }

  public void stopWrist(){
    m_wrist.set(0);
  }

  public void stopIntake(){
    m_intake.set(0);
  }

  public double getAngleDegrees(){
  double degrees;
  double percentage;
  percentage = absEncoder.get();
  degrees = percentage*360;
  return degrees;
  }

  public void initSendable(SendableBuilder builder){
    builder.addDoubleProperty("Arm Angle", ()->getAngleDegrees(), null);
    builder.addDoubleProperty("Raw Encoder",()-> absEncoder.get(), null);
  }
}

