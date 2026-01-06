// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import java.util.Map;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged
public class WinchSubsystem extends SubsystemBase {
  private DigitalInput input = new DigitalInput(9);
  private SparkMax m_winch = new SparkMax(8, MotorType.kBrushless);
  private DutyCycleEncoder winchEncoder = new DutyCycleEncoder(input, 360, Constants.WinchConstants.kWinchEncoderOffset);
  public GenericEntry nt_winchUpSpeed;
  public GenericEntry nt_winchDownSpeed;

  private double winchDesiredAngle;

  /** Creates a new WinchSubsystem. */
  public WinchSubsystem() {
    setupShuffleboard();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    checkWinchSoftLimits();
  }

  public Command retractWinch(){
    return this.startEnd(
      ()-> setWinchSpeed(getWinchUpSpeed()),
      ()-> winchStop());

  }

  public Command extendWinch(){
    return this.startEnd(
      ()-> setWinchSpeed(getWinchDownSpeed()),
      ()-> winchStop());
  }

  public void winchStop(){
    m_winch.stopMotor();
  }

  public void setWinchSpeed(double speed){
    m_winch.set(speed);
  }

  private void checkWinchSoftLimits() {
    if(goingUp() && maximumLimitReached()){
      winchStop();
    }

    if(goingDown() && minimumLimitReached()){
      winchStop();
    }
  };

  public boolean goingUp(){
    return m_winch.get() < 0;
  }  

  public boolean goingDown(){
    return m_winch.get() > 0;
  }  

  public boolean maximumLimitReached(){
    double degrees = this.getWinchAngle();
    boolean limitHit = true;
    if (degrees < Constants.WinchConstants.kWinchMax){
      limitHit = false;
    }
    return limitHit;
  }

  public boolean minimumLimitReached(){
    double degrees = this.getWinchAngle();
    boolean limitHit = false;
    if (degrees < Constants.WinchConstants.kWinchMin || degrees > 340.0){
      limitHit = true;
    }
    return limitHit;
  }

  public double getWinchAngle(){
    return winchEncoder.get();
  }

  public void setWinchAngle(double desiredAngle){
    winchDesiredAngle = desiredAngle;
  }

  protected boolean winchAtDesiredAngle() {
    // see if we're "close enough" to the target desired angle
    if (Math.abs(getWinchAngle() - winchDesiredAngle) <= Constants.WinchConstants.kWinchTolerance) {
      return true;
    }
    return false;
  }

  public void moveWinchToDesiredAngle() {

    if (winchAtDesiredAngle()) {
      // we've reached the goal angle, hold now
      setWinchSpeed(0);
    }
    else if (getWinchAngle() < winchDesiredAngle) {
      // need to move up to desired angle
      setWinchSpeed(getWinchUpSpeed());
    }
    else if (getWinchAngle() > winchDesiredAngle) {
      // move down to desired angle
      setWinchSpeed(getWinchDownSpeed());
    }
    
  }

    public Command cmdWinchToPositionThatFinishes(double angle) {
      winchDesiredAngle = angle;
    return new FunctionalCommand(
      // command start
      () -> setWinchAngle(winchDesiredAngle),
      // called repeatedly
      () -> moveWinchToDesiredAngle(),
      // Stop driving at the end of the command
      interrupted -> m_winch.set(0),
      // End the command when the robot's driven distance exceeds the desired value
      () -> winchAtDesiredAngle(),
      // Require the drive subsystem
      this
    );
  } 
  
  public double getWinchUpSpeed() {
        double elbowUpSpeed = nt_winchUpSpeed.getDouble(Constants.ArmConstants.kElbowUpSpeed);
      return elbowUpSpeed;
    } 

    public double getWinchDownSpeed() {
      double elbowDownSpeed = nt_winchDownSpeed.getDouble(Constants.ArmConstants.kElbowDownSpeed);
      return -1.0*elbowDownSpeed;
    } 

  private void setupShuffleboard() {

    ShuffleboardTab winchTab = Shuffleboard.getTab("Winch");

    nt_winchUpSpeed = winchTab.addPersistent("Winch Up speed", Constants.ArmConstants.kElbowUpSpeed)
        .withSize(3, 1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1))
        .getEntry();

    nt_winchDownSpeed = winchTab.addPersistent("Winch Down speed", Constants.ArmConstants.kElbowDownSpeed)
        .withSize(3, 1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1))
        .getEntry();

  }
}

