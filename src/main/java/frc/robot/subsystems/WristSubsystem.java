// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged
public class WristSubsystem extends SubsystemBase {

  private final DigitalInput inputWrist = new DigitalInput(8);
  private final DutyCycleEncoder wristEncoder = new DutyCycleEncoder(inputWrist, 360,
      Constants.ArmConstants.kWristAngleOffset);
  private final SparkMax m_wrist = new SparkMax(7, MotorType.kBrushless);
  public double wristDesiredAngleDeg = 90.0;
  private GenericEntry nt_wristSpeed;

  private PIDController wrist_pid = new PIDController(0.01, 0, 0);
  public static double CENTER_REVERSE = 315;
  public static double CENTER = 135;
  public static double LEFT = 45;
  public static double RIGHT = 220;

  /** Creates a new ExampleSubsystem. */
  public WristSubsystem() {
    wristEncoder.setInverted(true);
    setupShuffleboard();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    checkWristSoftLimits();
    ProfileEndMotion();
  }

  private void checkWristSoftLimits() {
    if (goingLeft() && leftLimitReached()) {
      setWristSpeed(0);
    }

    if (goingRight() && rightLimitReached()) {
      setWristSpeed(0);
    }
  };

  public boolean leftLimitReached() {
    double degrees = this.getWristAngle();
    boolean limitHit = true;
    if (degrees > Constants.ArmConstants.kWristLeftLimit) {
      limitHit = false;
      // System.out.println("lower limit exceed:"+degrees);
    }
    return limitHit;
  }

  public boolean rightLimitReached() {
    double degrees = this.getWristAngle();
    boolean limitHit = true;
    if (degrees < Constants.ArmConstants.kWristRightLimit) {
      limitHit = false;
      // System.out.println("lower limit exceed:"+degrees);
    }
    return limitHit;
  }

  public boolean goingLeft() {
    return m_wrist.get() < 0;
  }

  public boolean goingRight() {
    return m_wrist.get() > 0;
  }

  public Command manualWristLeft() {
    return this.startEnd(
        // Command Start
        () -> setWristSpeed(-getWristSpeed()),
        // Command End
        () -> m_wrist.stopMotor()).withName("Manual Left");
  }

  public Command manualWristRight() {
    return this.startEnd(
        // Command Start
        () -> setWristSpeed(getWristSpeed()),
        // Command End
        () -> m_wrist.stopMotor()).withName("Manual Right");
  }




  public Command moveWristLeft() {
    return moveWristToPosition(WristSubsystem.LEFT).withName("MoveLeft");
  }

  public Command moveWristCenter() {
    return moveWristToPosition(WristSubsystem.CENTER).withName("MoveCenter");
  }

  public Command moveWristRight() {
    return moveWristToPosition(WristSubsystem.RIGHT).withName("MoveRight");
  }


  public Command moveWristToPosition(double p) {
    return this.startRun(
      ()->setWristDesiredAngle(p),
      ()->moveWristWithPID())
      .until(() -> wristAtDesiredAngle())
      .finallyDo(() -> m_wrist.stopMotor())
      .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }

  public double getWristAngle() {
    return wristEncoder.get();
  }

  public boolean isPieceVertical(){
    return !MathUtil.isNear(WristSubsystem.CENTER, getWristAngle(), 20);
  }

  private double getWristSpeed() {
    return nt_wristSpeed.getDouble(0.1);
  }

  public void setWristDesiredAngle(double angle) {
    wristDesiredAngleDeg = angle;
    wrist_pid.reset(); //clears previous state
  }

  public void moveWristWithPID(){
    if(!wristEncoder.isConnected()){
      System.out.println("ERROR - Wrist Encoder is not connected.");
      setWristSpeed(0);
    }
    double updated_speed = MathUtil.clamp(wrist_pid.calculate(getWristAngle(), wristDesiredAngleDeg), -getWristSpeed(), getWristSpeed());
    setWristSpeed(updated_speed);
  }

  public void moveWristToDesiredAngle() {

    if (wristAtDesiredAngle()) {
      // we've reached the goal angle, stop
      setWristSpeed(0);
    } else if (getWristAngle() < wristDesiredAngleDeg) {
      // need to move left to desired angle
      setWristSpeed(getWristSpeed());
    } else if (getWristAngle() > wristDesiredAngleDeg) {
      // move right to desired angle
      setWristSpeed(-getWristSpeed());
    }

  }

  public String currentCommand(){
    if(this.getCurrentCommand() != null){
      return this.getCurrentCommand().getName();
    } else {
      return "None";
    }
  }
  
  @Logged
  public boolean wristAtDesiredAngle() {
    // see if we're "close enough" to the target desired angle
    if (Math.abs(getWristAngle() - wristDesiredAngleDeg) <= Constants.ArmConstants.kWristAngleToleranceDeg) {
      return true;
    }
    return false;
  }

  private void setWristSpeed(double speed) {
    m_wrist.set(speed);
    checkWristSoftLimits(); // check here to avoid a loop delay in checking
  }

  private void setupShuffleboard() {

    ShuffleboardTab tab = Shuffleboard.getTab("Wrist");

    nt_wristSpeed = tab.addPersistent("Wrist speed", 0.1)
        .withSize(3, 1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1))
        .getEntry();
        tab.add(moveWristLeft());
        tab.add(moveWristCenter());
        tab.add(moveWristRight());

    tab.add(wrist_pid);

  }


  // End is the end of motion
  // Start is the start of the motion limiting.
  private double TopEnd = Constants.ArmConstants.kWristRightLimit;
  private double TopStart = Constants.ArmConstants.kWristRightLimit-70;
  private double BottomEnd = Constants.ArmConstants.kWristLeftLimit;
  private double BottomStart = Constants.ArmConstants.kWristLeftLimit+70;
  
 
    private void ProfileEndMotion(){
      //this method is used to automaticaly profile motion as the elevator approaches it's limits.
  
      //if in upper or lower ranges, then set the max speed based on percentage
      double limit = speedLimitAtCurrentPosition();
      double motor_speed = m_wrist.get();
      if(limit < Math.abs(motor_speed)){
        m_wrist.set(MathUtil.clamp(motor_speed, -limit, limit));
      }
  
    }
  
    public double speedLimitAtCurrentPosition(){
      double position = getWristAngle();
      double motor_direction = m_wrist.get();
      if(TopEnd > position && position > TopStart && motor_direction > 0){ //negative motion is up
        return upperRangePercentage();
      }else if(BottomStart > position && position > BottomEnd && motor_direction < 0){ //positive motion is down.
        return lowerRangePercentage();
      }
      return 1; //no speed limit at current position.
  
    }
  
    public double upperRangePercentage(){
      return calculateMotionProfilePercentage(TopEnd, TopStart, getWristAngle());
    }
  
    public double lowerRangePercentage(){
      return calculateMotionProfilePercentage(BottomEnd, BottomStart, getWristAngle());
    }
  
    public static double calculateMotionProfilePercentage(double start, double end, double current) {
      // Calculate the absolute difference between start and end
      double total = Math.abs(end - start);
      
      // Calculate the difference between current and start
      double progress = Math.abs(current - start);
      
      // Calculate the percentage
      double percentage = (progress / total);
      
      // Ensure the percentage is between 0 and 100
      percentage = Math.min(1, Math.max(0, percentage));
      
      return percentage;
    }
  


}
