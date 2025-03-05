// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

@Logged
public class ArmSubsystem extends SubsystemBase {

  private SparkMaxConfig config= new SparkMaxConfig();
  private final DigitalInput inputElbow = new DigitalInput(7);

  private final DutyCycleEncoder absEncoder = new DutyCycleEncoder(inputElbow, -1.0, -Constants.ArmConstants.kElbowOffset);
  private final SparkMax m_elbow = new SparkMax(6, MotorType.kBrushless);
  private boolean maxLimitReached = false;
  private boolean minLimitReached = false;
  //private double m_elbowSpeed = 0.2;
  //private double m_elbowUpSpeed = Constd;
  //private double m_elbowDownSpeed = Constants.ArmConstants.kElbowDownSpeed;  // make vars to allow tuning
  
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
    absEncoder.setInverted(true);
    absEncoder.setDutyCycleRange(-180, 180);

  }

  @Override
  public void periodic() {
    
    /*if(m_autoElbowEnabled){
      moveArmToDesiredAngle();
    };*/
    
    checkElbowSoftLimits();

  }

  //@Logged
  //public String currentCommandName(){
  //  return this.getCurrentCommand().getName();
  //}
  /*
   *  Check if the elbow is exceeding angle limits.  If so and it's going the wrong direction, stop the motor before doing any damage
   */
  private void checkElbowSoftLimits() {
    if(goingUp() && maximumLimitReached()){
      m_elbow.stopMotor();
    }

    if(goingDown() && minimumLimitReached()){
      m_elbow.stopMotor();
    }
  };

  public boolean maximumLimitReached(){
    double degrees = this.getArmAngle();
    boolean limitHit = true;
    if (degrees < Constants.ArmConstants.kMaxElbowAngle){
      limitHit = false;
      //System.out.println("lower limit exceed:"+degrees);
    }
    return limitHit;
  }

  public boolean minimumLimitReached(){
    double degrees = this.getArmAngle();
    boolean limitHit = true;
    if (degrees > Constants.ArmConstants.kMinElbowAngle){
      limitHit = false;
      //System.out.println("lower limit exceed:"+degrees);
    }
    return limitHit;
  }


    public boolean goingUp(){
      return m_elbow.get() > Constants.ArmConstants.kElbowHoldSpeed;
    }  

    public boolean goingDown(){
      return m_elbow.get() < 0;
    }  


  public Command manualElbowUp(){
    return this.startEnd(
      // Command Start
      () -> elbowUp(),
      // Command End
      () -> elbowHold()
    );
  }

  public Command manualElbowDown(){
    return this.startEnd(
      // Command Start
      () -> elbowDown(),
      // Command End
      () -> elbowHold()
    );
  }

  public Command cmdArmHorizontal() {
    return this.startRun(
      ()->setArmAngle(0.0),
      ()->moveArmToDesiredAngle());
  } 

  public Command cmdArmAngle(double angle){
    return this.startRun(
      ()->setArmAngle(angle),
      ()->moveArmToDesiredAngle()
    );
  }
 
  public Command cmdArmHorizontalThatFinishes() {
    return new FunctionalCommand(
      // command start
      () -> setArmAngle(0.0),
      // called repeatedly
      () -> moveArmToDesiredAngle(),
      // Stop driving at the end of the command
      interrupted -> m_elbow.set(Constants.ArmConstants.kElbowHoldSpeed),
      // End the command when the robot's driven distance exceeds the desired value
      () -> elbowAtDesiredAngle(),
      // Require the drive subsystem
      this
    );
  } 
  
  
  public Command cmdArmPIDHorizontal() {
    return this.startRun(
      ()->setArmAngle(0.0),
      ()->moveArmWithPIDFF());
    }

  /*
   * either moves the elbow/arm to the desired angle, or hold it there if at angle
   */
  public void moveArmToDesiredAngle() {
 
    if (elbowAtDesiredAngle()) {
      // we've reached the goal angle, hold now
      m_elbow.set(Constants.ArmConstants.kElbowHoldSpeed);
    }
    else if (getArmAngle() < m_elbowDesiredAngleDeg ) {
      // need to move up to desired angle
      setElbowSpeed(getElbowUPSpeed());
    }
    else if (getArmAngle() > m_elbowDesiredAngleDeg) {
      // move down to desired angle
      setElbowSpeed(getElbowDOWNSpeed());
    }
    
  }


  protected boolean elbowAtDesiredAngle() {
    // see if we're "close enough" to the target desired angle
    if (Math.abs(getArmAngle() - m_elbowDesiredAngleDeg) <= Constants.ArmConstants.kElbowAngleToleranceDeg) {
      // close enough
  //    System.out.println(" At requested elbow angle:"+getElbowAngleDegrees());
      return true;
    }
    return false;
  }

  protected void moveArmWithPIDFF() {
    // calculate movement with fancy-pants PID and feedforward
    //  As of 2/25 UNUSED
    //feedforward = m_elbowFeedforward.calculate(m_elbowDesiredAngleDeg,0.0);
    // convert feedforward from nominal voltage to %, based on current voltage. 
    // hopefully this compensates for battery depletion to still supply the correct voltage to maintain position
    //feedforward = feedforward / m_elbow.getBusVoltage();
    if (elbowAtDesiredAngle()) return;

    pidOutput = 0.0;
      pidOutput = m_elbowPIDController.calculate(getArmAngle());
     
    if (target_speed > 0.0) target_speed = getElbowUPSpeed();
    if (target_speed < 0.0) target_speed = getElbowDOWNSpeed();
    
    //limit max speed at point of applying to motor.
    target_speed = Math.min(target_speed,pidOutput);

    setElbowSpeed(target_speed);
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
      setElbowSpeed(getElbowUPSpeed());
   
  }

  public void elbowDown() {
      setElbowSpeed(getElbowDOWNSpeed());
  }

  private void elbowHold(){
    // I've bypassed the setElbowSpeed method to avoid all the calculations there. This can use that method if we test it to work.
    m_elbow.set(Constants.ArmConstants.kElbowHoldSpeed);
  }

  public void stopElbow() {
    m_elbow.set(0);
    setArmAngle(getArmAngle()); // hold that position
  }
  
  public double getRawElbowAngleDegrees() {

    m_percentage = absEncoder.get();
    m_degrees = m_percentage * 360;
    return m_degrees;
  }

  public double getArmAngle(){
    double d = getRawElbowAngleDegrees();
    if(d < -180){
      d = d + 360;
      return d;
    }
    return d;
  }

  private void setupShuffleboard() {

    ShuffleboardTab armTab = Shuffleboard.getTab("Arm");

    nt_elbowUPSpeed = armTab.addPersistent("Elbow UP speed", Constants.ArmConstants.kElbowUpSpeed)
        .withSize(3, 1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1))
        .getEntry();

    nt_elbowDOWNSpeed = armTab.addPersistent("Elbow DOWN speed", Constants.ArmConstants.kElbowDownSpeed)
        .withSize(3, 1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1))
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
    /*if (setspeed > 0 && maximumLimitReached()){
      return;
    }

    if (setspeed < 0 && minimumLimitReached()){
      return;
    }*/
/* 
    m_autoElbowEnabled = true; // we've requested arm movement, enable it 
    double dampenedSpeed = setspeed;
    // dial down the requested speed if we are close to the target angle
    if (Math.abs(getArmAngle() - m_elbowDesiredAngleDeg) <= (3.0*Constants.ArmConstants.kElbowAngleToleranceDeg)) {
      dampenedSpeed = 0.5*setspeed;
    }
    double degrees = this.getArmAngle();
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
      */
      m_elbow.set(setspeed);
  }

  public double getElbowUPSpeed() {

  //  if (m_elbowUpSpeed != nt_elbowUPSpeed.getDouble(Constants.ArmConstants.kElbowUpSpeed)) {
      // get the value from the Shuffleboard slider.  If it changed salt it away for future reboots
      double elbowUpSpeed = nt_elbowUPSpeed.getDouble(Constants.ArmConstants.kElbowUpSpeed);
    //  Preferences.setDouble(Constants.ArmConstants.kElbowUpSpeedPrefKey, m_elbowUpSpeed);
    //}
    return elbowUpSpeed;
  } 
  public double getElbowDOWNSpeed() {

 //   if (m_elbowDownSpeed != nt_elbowDOWNSpeed.getDouble(Constants.ArmConstants.kElbowDownSpeed)) {
      // get the value from the Shuffleboard slider.  If it changed salt it away for future reboots
      double elbowDownSpeed = nt_elbowDOWNSpeed.getDouble(Constants.ArmConstants.kElbowDownSpeed);
   //   Preferences.setDouble(Constants.ArmConstants.kElbowDownSpeedPrefKey, m_elbowDownSpeed);
   // }
    return -1.0*elbowDownSpeed;
  } 


  public void initSendable(SendableBuilder builder){
    builder.addDoubleProperty("Raw Absolute Encoder", ()-> m_percentage, null);
    builder.addDoubleProperty("Absolute Encoder Degrees", ()-> m_degrees, null); 
    builder.addDoubleProperty("Feed Forward", ()-> feedforward, null);
    builder.addDoubleProperty("PID Output", ()-> pidOutput, null);
    builder.addDoubleProperty("Voltage", ()-> target_speed, null);
  }
}

