// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged
public class WristSubsystem extends SubsystemBase {

  private final DigitalInput inputWrist = new DigitalInput(8);
  private final DutyCycleEncoder wristEncoder = new DutyCycleEncoder(inputWrist, -1.0, -Constants.ArmConstants.kWristAngleOffset);
  private final SparkMax m_wrist = new SparkMax(7, MotorType.kBrushless);
  public double wristDesiredAngleDeg = 90.0;
  private GenericEntry nt_wristSpeed;

  /** Creates a new ExampleSubsystem. */
  public WristSubsystem() {
    wristEncoder.setInverted(true);
    setupShuffleboard();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
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
    checkWristSoftLimits();
  }


  private void checkWristSoftLimits() {
    if(goingLeft() && leftLimitReached()){
      setWristSpeed(0);
    }

    if(goingRight() && rightLimitReached()){
      setWristSpeed(0);;
    }
  };

  public boolean leftLimitReached(){
    double degrees = this.getWristAngle();
    boolean limitHit = true;
    if (degrees > Constants.ArmConstants.kWristLeftLimit){
      limitHit = false;
      //System.out.println("lower limit exceed:"+degrees);
    }
    return limitHit;
  }

  public boolean rightLimitReached(){
    double degrees = this.getWristAngle();
    boolean limitHit = true;
    if (degrees < Constants.ArmConstants.kWristRightLimit){
      limitHit = false;
      //System.out.println("lower limit exceed:"+degrees);
    }
    return limitHit;
  }

  public boolean goingLeft(){
    return m_wrist.get() > 0;
  }

  public boolean goingRight(){
    return m_wrist.get() < 0;
  }

  public Command manualWristCW(){
    return this.startEnd(
      // Command Start
      () -> setWristSpeed(getWristSpeed()),
      // Command End
      () -> m_wrist.stopMotor()
      );
  }
    
  public Command manualWristCCW(){
    return this.startEnd(
      // Command Start
       () -> setWristSpeed(-getWristSpeed()),
       // Command End
       () -> m_wrist.stopMotor()
       );
    }

    
      
    public Command moveWristLeft(){
      setWristDesiredAngle(-90);
      return new InstantCommand(()-> moveWristToDesiredAngle());
    }
      
    public Command moveWristCenter(){
      setWristDesiredAngle(0);
      return new InstantCommand(()-> moveWristToDesiredAngle());
    }
      
    public Command moveWristRight(){
      setWristDesiredAngle(90);
      return new InstantCommand(()-> moveWristToDesiredAngle());
      }
      
    public double getWristAngle(){
      double angle = wristEncoder.get();
      angle = angle * -1;
      angle = angle * 360 - Constants.ArmConstants.kWristAngleOffset;
      return angle;
    }
      
    private double getWristSpeed(){
      return nt_wristSpeed.getDouble(0.1);
    }

    
  public void setWristDesiredAngle(double angle){
    wristDesiredAngleDeg = angle;
  }

  public void moveWristToDesiredAngle() {
 
    if (wristAtDesiredAngle()) {
      // we've reached the goal angle, stop
     setWristSpeed(0);
    }
    else if (getWristAngle() < wristDesiredAngleDeg ) {
      // need to move left to desired angle
      setWristSpeed(-0.1);
    }
    else if (getWristAngle() > wristDesiredAngleDeg) {
      // move right to desired angle
      setWristSpeed(0.4);
    }
    
  }

  protected boolean wristAtDesiredAngle() {
    // see if we're "close enough" to the target desired angle
    if (Math.abs(getWristAngle() - wristDesiredAngleDeg) <= Constants.ArmConstants.kWristAngleToleranceDeg) {
      return true;
    }
    return false;
  }

  private void setWristSpeed(double speed){
    m_wrist.set(speed);
  }

  public void wristLeft(){
    m_wrist.set(0.1);
  }

  public void wristRight(){
    m_wrist.set(-0.1);
  }

  public void stopWrist(){
    m_wrist.set(0);
  }

    private void setupShuffleboard() {

    ShuffleboardTab tab = Shuffleboard.getTab("Wrist");

    nt_wristSpeed = tab.addPersistent("Wrist speed", 0.1)
        .withSize(3, 1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1))
        .getEntry();

  }

}
