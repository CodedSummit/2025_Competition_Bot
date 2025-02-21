// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

  private SparkMaxConfig config= new SparkMaxConfig();
  private final DigitalInput input = new DigitalInput(7);
  private final DutyCycleEncoder absEncoder = new DutyCycleEncoder(input);
  private final SparkMax m_elbow = new SparkMax(6, MotorType.kBrushless);
  private final SparkMax m_wrist = new SparkMax(7, MotorType.kBrushless);
  private final SparkMax m_intake = new SparkMax(8, MotorType.kBrushless);
  private boolean maxLimitReached = false;
  private boolean minLimitReached = false;
  private double m_elbowSpeed = 0.2;
  private double m_elbowDesiredAngleDeg = 0.0;  // Angle we want the arm.  0.0 is horizontal, 90 straight up

 //private final Encoder m_encoder = new Encoder(ArmConstants.kEncoderPorts[0], ArmConstants.kEncoderPorts[1]);
  private  ArmFeedforward m_elbowFeedforward = new ArmFeedforward(
      ArmConstants.kSVolts, ArmConstants.kGVolts,
      ArmConstants.kVVoltSecondPerRad);
  private PIDController m_elbowPIDController = new PIDController(ArmConstants.kP,0,0);
  private double m_handlerSpeed = ArmConstants.kHandlerDefaultSpeed;
  private GenericEntry nt_elbowSpeed;





  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {

    config.apply(config);
    setupShuffleboard();
    m_elbowPIDController.setTolerance(ArmConstants.kElbowAngleToleranceDeg);
  }

  @Override
  public void periodic() {
    moveArmToDesiredAngle();
    checkElbowSoftLimits();
  }

  /*
   *  Check if the elbow is exceeding angle limits.  If so, stop the motor before doing any damage
   */
  private void checkElbowSoftLimits() {
    // TODO  - log some messages if limits are exceeded
    double degrees = this.getElbowAngleDegrees();
    if (degrees > Constants.ArmConstants.kMaxElbowAngle){
      maxLimitReached = true;
      if(m_elbow.get() < 0){
        m_elbow.stopMotor();
      }
    }else{
      maxLimitReached = false;
    }

    if (degrees<Constants.ArmConstants.kMinElbowAngle-20){  // TODO - what is the subtraction of 20 for????
      minLimitReached = true;
      if(m_elbow.get() > 0){
        m_elbow.stopMotor();
      }
    }else{
      minLimitReached = false;
    }
  }

  /*
   * either moves the elbow/arm to the desired angle, or hold it there if at angle
   */
  public void moveArmToDesiredAngle() {
       
    double feedforward = m_elbowFeedforward.calculate(m_elbowDesiredAngleDeg,0.0);
    double pidOutput = 0.0;
    if (!m_elbowPIDController.atSetpoint()) {
      pidOutput = m_elbowPIDController.calculate(getElbowAngleDegrees());
    }
     
    // Add the feedforward to the PID output to get the motor output
    double target_voltage = pidOutput + feedforward;
    
    //limit max voltage at point of applying to motor.
    target_voltage = Math.min(target_voltage, 0.8);

    m_elbow.setVoltage(target_voltage);
    
  }

  // move the arm to a horizontal position
  public void setArmHorizontal() {
    setArmAngle(0.0);
  }

  // use PID positioning to bump arm up/down
  public void bumpArmUp() {
    setArmAngle(m_elbowDesiredAngleDeg + ArmConstants.kArmBumpIncrementDeg);
  }

   public void bumpArmDown() {
    setArmAngle(m_elbowDesiredAngleDeg - ArmConstants.kArmBumpIncrementDeg);
  }



  public void elbowMove(){
      m_elbow.set(getElbowSpeed());
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

  public double getElbowAngleDegrees() {
    double degrees;
    double percentage;
    percentage = absEncoder.get();
    degrees = percentage * 360 - Constants.ArmConstants.kElbowOffset;

    return degrees;
  }
  private void setupShuffleboard() {

    ShuffleboardTab armTab = Shuffleboard.getTab("Arm");

    nt_elbowSpeed = armTab.addPersistent("Elbow speed", m_elbowSpeed)
        .withSize(3, 1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -1, "max", 1))
        .getEntry();

  
  }

  private void setArmAngle(double desiredAngle) {
    m_elbowDesiredAngleDeg = desiredAngle;
    m_elbowPIDController.setSetpoint(desiredAngle);
  }

  public double getElbowSpeed() {

    if (m_elbowSpeed != nt_elbowSpeed.getDouble(Constants.ArmConstants.kElbowSpeed)) {
      // get the value from the Shuffleboard slider.  If it changed salt it away for future reboots
      m_elbowSpeed = nt_elbowSpeed.getDouble(Constants.ArmConstants.kElbowSpeed);
      Preferences.setDouble(Constants.ArmConstants.kElbowSpeedPrefKey, m_elbowSpeed);
    }
    return m_elbowSpeed;
  } 

  public void initSendable(SendableBuilder builder){
    builder.addDoubleProperty("Arm Angle", ()->getElbowAngleDegrees(), null);
    builder.addDoubleProperty("Raw Encoder",()-> absEncoder.get(), null);
  }
}

