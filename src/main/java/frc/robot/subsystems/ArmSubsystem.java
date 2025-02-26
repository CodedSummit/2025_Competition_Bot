// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.Logged;
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

@Logged
public class ArmSubsystem extends SubsystemBase {

  private SparkMaxConfig config= new SparkMaxConfig();
  private final DigitalInput input = new DigitalInput(7);
  private final DutyCycleEncoder absEncoder = new DutyCycleEncoder(input);
  private final SparkMax m_elbow = new SparkMax(6, MotorType.kBrushless);
  private final SparkMax m_wrist = new SparkMax(7, MotorType.kBrushless);
  private final SparkMax m_hand = new SparkMax(8, MotorType.kBrushed);
  private boolean maxLimitReached = false;
  private boolean minLimitReached = false;
  private double m_elbowSpeed = 0.2;
  private double m_elbowUpSpeed = Constants.ArmConstants.kElbowUpSpeed;
  private double m_elbowDownSpeed = Constants.ArmConstants.kElbowDownSpeed;  // make vars to allow tuning
  
  private double m_elbowDesiredAngleDeg = 0.0;  // Angle we want the arm.  0.0 is horizontal, 90 straight up

 //private final Encoder m_encoder = new Encoder(ArmConstants.kEncoderPorts[0], ArmConstants.kEncoderPorts[1]);
  private  ArmFeedforward m_elbowFeedforward = new ArmFeedforward(
      ArmConstants.kSVolts, ArmConstants.kGVolts,
      ArmConstants.kVVoltSecondPerRad);
  private PIDController m_elbowPIDController = new PIDController(ArmConstants.kP,0,0);
  private double m_handlerSpeed = ArmConstants.kHandlerDefaultSpeed;
  private GenericEntry nt_elbowSpeed;
  private GenericEntry nt_elbowUPSpeed;
  private GenericEntry nt_elbowDOWNSpeed;
  private double m_percentage;
  private double m_degrees;
  private boolean m_autoElbowEnabled = false;
  private double feedforward;
  private double pidOutput;
  private double target_speed;  // fom -1.0 to 1.0

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {

    config.apply(config);
    setupShuffleboard();
    m_elbowPIDController.setTolerance(ArmConstants.kElbowAngleToleranceDeg);
  }

  @Override
  public void periodic() {
    
    if(m_autoElbowEnabled){
      moveArmToDesiredAngle();
    };
    
    checkElbowSoftLimits();
    
  }

  /*
   *  Check if the elbow is exceeding angle limits.  If so and it's going the wrong direction, stop the motor before doing any damage
   */
  private void checkElbowSoftLimits() {
    // TODO  - log some messages if limits are exceeded
    double degrees = this.getElbowAngleDegrees();
    if (degrees > Constants.ArmConstants.kMaxElbowAngle){
      maxLimitReached = true;
  //    System.out.println("upper limit exceed:"+degrees);
      if(m_elbow.get() > 0){
        m_elbow.stopMotor();
      }
    }else{
      maxLimitReached = false;
    }

    if (degrees<Constants.ArmConstants.kMinElbowAngle){
      minLimitReached = true;
      //System.out.println("lower limit exceed:"+degrees);

      if(m_elbow.get() < 0){
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
 
    if (elbowAtDesiredAngle()) {
      // we've reached the goal angle, hold now
      setElbowSpeed(Constants.ArmConstants.kElbowHoldSpeed);
    }

    if (getElbowAngleDegrees() < m_elbowDesiredAngleDeg ) {
      // need to move up to desired angle
      setElbowSpeed(getElbowUPSpeed());
    }
    else if (getElbowAngleDegrees() > m_elbowDesiredAngleDeg) {
      // move down to desired angle
      setElbowSpeed(getElbowDOWNSpeed());
    }
    
  }

  protected boolean elbowAtDesiredAngle() {
    // see if we're "close enough" to the target desired angle
    if (Math.abs(getElbowAngleDegrees() - m_elbowDesiredAngleDeg) <= Constants.ArmConstants.kElbowAngleToleranceDeg) {
      // close enough
      return true;
    }
    return false;
  }

  protected void moveArmWithPIDFF() {
    // calculate movement with fancy-pants PID and feedforward
    //  As of 2/25 UNUSED
    feedforward = m_elbowFeedforward.calculate(m_elbowDesiredAngleDeg,0.0);
    // convert feedforward from nominal voltage to %, based on current voltage. 
    // hopefully this compensates for battery depletion to still supply the correct voltage to maintain position
    feedforward = feedforward / m_elbow.getBusVoltage();
    pidOutput = 0.0;
      pidOutput = m_elbowPIDController.calculate(getElbowAngleDegrees());
     
    // Add the feedforward to the PID output to get the motor output
    target_speed = pidOutput + feedforward;
    
    //limit max speed at point of applying to motor.
    target_speed = Math.min(target_speed, 0.05);

    setElbowSpeed(target_speed);
  }


  // move the arm to a horizontal position
  public void setArmHorizontal() {
    setArmAngle(0.0);
    m_autoElbowEnabled = true;
  }

  // use PID positioning to bump arm up/down
  public void bumpArmUp() {
    setArmAngle(m_elbowDesiredAngleDeg + ArmConstants.kArmBumpIncrementDeg);
  }

  public void bumpArmDown() {
    setArmAngle(m_elbowDesiredAngleDeg - ArmConstants.kArmBumpIncrementDeg);
  }

/*
 * Manually drive the elbow up or down.  When stopped, it will use the control 
 * loop to hold that new position
 */

  public void elbowUp() {
    if (!maxLimitReached) {
      setElbowSpeed(m_elbowUpSpeed);
    }
  }

  public void elbowDown() {
    if (!minLimitReached) {
      setElbowSpeed(m_elbowDownSpeed);
    }  
  }

  public void stopElbow() {
    m_elbow.set(0);
    setArmAngle(getElbowAngleDegrees()); // hold that position
  }
  
  public void wristLeft(){
    m_wrist.set(0.1);
  }

  public void wristRight(){
    m_wrist.set(-0.1);
  }

  public void intakeCoral(){
    m_hand.set(0.1);
    new WaitCommand(2);
    m_hand.set(0);
  }

  public void placeCoral(){
    m_hand.set(-0.1);
    new WaitCommand(2);
    m_hand.set(0);
  }



  public void stopWrist(){
    m_wrist.set(0);
  }

  public void stopIntake(){
    m_hand.set(0);
  }

  public double getElbowAngleDegrees() {

    m_percentage = absEncoder.get();
    m_degrees = m_percentage * 360 - Constants.ArmConstants.kElbowOffset;
    return m_degrees;
  }
  private void setupShuffleboard() {

    ShuffleboardTab armTab = Shuffleboard.getTab("Arm");

    nt_elbowSpeed = armTab.addPersistent("Elbow speed", m_elbowSpeed)
        .withSize(3, 1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 2))
        .getEntry();

    nt_elbowUPSpeed = armTab.addPersistent("Elbow UP speed", m_elbowUpSpeed)
        .withSize(3, 1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 2))
        .getEntry();

    nt_elbowDOWNSpeed = armTab.addPersistent("Elbow DOWN speed", m_elbowDownSpeed)
        .withSize(3, 1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -2, "max", 0))
        .getEntry();

  }

  private void setArmAngle(double desiredAngle) {
    m_elbowDesiredAngleDeg = desiredAngle;
    m_elbowPIDController.setSetpoint(desiredAngle);
  }

  /*
   * set to the requested speed, but only if it won't violate soft limits
   * Else make no change to the motor speed
   */
  protected void setElbowSpeed(double setspeed) {
    m_autoElbowEnabled = true; // we've requested arm movement, enable it 
    double dampenedSpeed = setspeed;
    // dial down the requested speed if we are close to the target angle
    if (Math.abs(getElbowAngleDegrees() - m_elbowDesiredAngleDeg) <= (3.0*Constants.ArmConstants.kElbowAngleToleranceDeg)) {
      dampenedSpeed = 0.5*setspeed;
    }
    double degrees = this.getElbowAngleDegrees();
    if ((degrees < Constants.ArmConstants.kMaxElbowAngle) &&
        (degrees > Constants.ArmConstants.kMinElbowAngle)) {
      // This is ok - not in limit so set the speed regardless of direction
      m_elbow.set(dampenedSpeed);
    } else if ((degrees > Constants.ArmConstants.kMaxElbowAngle) &&
        (dampenedSpeed < 0.0)) {
      // this is OK. Currently exceeding max limit but it's OK to move in direction
      // away from the limit
      m_elbow.set(dampenedSpeed);
    } else if ((degrees < Constants.ArmConstants.kMinElbowAngle) &&
        (dampenedSpeed > 0.0)) {
      // this is OK. Currently exceeding min limit but it's OK to move in direction
      // away from the limit
      m_elbow.set(dampenedSpeed);
    }
  }

  public double getElbowSpeed() {

    if (m_elbowSpeed != nt_elbowSpeed.getDouble(Constants.ArmConstants.kElbowSpeed)) {
      // get the value from the Shuffleboard slider.  If it changed salt it away for future reboots
      m_elbowSpeed = nt_elbowSpeed.getDouble(Constants.ArmConstants.kElbowSpeed);
      Preferences.setDouble(Constants.ArmConstants.kElbowSpeedPrefKey, m_elbowSpeed);
    }
    return m_elbowSpeed;
  } 
  public double getElbowUPSpeed() {

    if (m_elbowUpSpeed != nt_elbowUPSpeed.getDouble(Constants.ArmConstants.kElbowUpSpeed)) {
      // get the value from the Shuffleboard slider.  If it changed salt it away for future reboots
      m_elbowSpeed = nt_elbowUPSpeed.getDouble(Constants.ArmConstants.kElbowUpSpeed);
      Preferences.setDouble(Constants.ArmConstants.kElbowUpSpeedPrefKey, m_elbowUpSpeed);
    }
    return m_elbowUpSpeed;
  } 
  public double getElbowDOWNSpeed() {

    if (m_elbowDownSpeed != nt_elbowDOWNSpeed.getDouble(Constants.ArmConstants.kElbowDownSpeed)) {
      // get the value from the Shuffleboard slider.  If it changed salt it away for future reboots
      m_elbowDownSpeed = nt_elbowDOWNSpeed.getDouble(Constants.ArmConstants.kElbowDownSpeed);
      Preferences.setDouble(Constants.ArmConstants.kElbowDownSpeedPrefKey, m_elbowDownSpeed);
    }
    return m_elbowDownSpeed;
  } 
  public void initSendable(SendableBuilder builder){
    builder.addDoubleProperty("Raw Absolute Encoder", ()-> m_percentage, null);
    builder.addDoubleProperty("Absolute Encoder Degrees", ()-> m_degrees, null); 
    builder.addDoubleProperty("Feed Forward", ()-> feedforward, null);
    builder.addDoubleProperty("PID Output", ()-> pidOutput, null);
    builder.addDoubleProperty("Voltage", ()-> target_speed, null);
  }
}

