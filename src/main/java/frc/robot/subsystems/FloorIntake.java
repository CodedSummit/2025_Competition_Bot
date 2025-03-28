// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
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
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged
public class FloorIntake extends SubsystemBase {

  private final VictorSPX intakeArmMotor = new VictorSPX(11);
  private final VictorSPX intakeWheels = new VictorSPX(10);

  private final DigitalInput floorInput = new DigitalInput(9);
  private final DutyCycleEncoder intakeArmPosition = new DutyCycleEncoder(floorInput, 360, Constants.IntakeConstants.kIntakeArmOffset);

  private GenericEntry nt_armSpeedDown;
  private GenericEntry nt_armSpeedUp;

  //This needs a FeedForward to hit a specific setpoint
    private PIDController intake_arm_pid = new PIDController(0.005, 0, 0);
    private double intakeArmDesiredAngle = 180;

  public static double UP_POSITION = 180;
  public static double ALGEA_POSITION = 220;
  public static double FLOOR_INTAKE_POSITION = 240;

  /** Creates a new FloorIntake. */
  public FloorIntake() {
    intakeWheels.setInverted(true);
    this.initialize();
  }


    public void initialize() {
      setupShuffleboard();
    }


  public double armPosition(){
    return intakeArmPosition.get();
  }

  @Logged
  public Command ManualArmIn(){
    return this.startEnd(
      () -> intakeArmMotor.set(VictorSPXControlMode.PercentOutput, getArmSpeedUp()), 
      () -> intakeArmMotor.set(VictorSPXControlMode.PercentOutput, 0)
    );
  }

  @Logged
  public Command ManualArmOut(){
    return this.startEnd(
      () -> intakeArmMotor.set(VictorSPXControlMode.PercentOutput, -1 * getArmSpeedDown()), 
      () -> intakeArmMotor.set(VictorSPXControlMode.PercentOutput, 0)
    );
  }


  public void setIntakeArmDesiredAngle(double angle) {
    intakeArmDesiredAngle = angle;
    intake_arm_pid.reset(); //clears previous state
  }

  public Command moveArmToPosition(double p) {
    return this.startRun(
      ()->setIntakeArmDesiredAngle(p),
      ()->moveIntakeArmWithPID())
      .withTimeout(1)
      .finallyDo(() -> stopArm())
      .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }

  public void moveIntakeArmWithPID(){
    double updated_speed = MathUtil.clamp(intake_arm_pid.calculate(armPosition(), intakeArmDesiredAngle), -getArmSpeedDown(), getArmSpeedUp());
    intakeArmMotor.set(VictorSPXControlMode.PercentOutput, -updated_speed);
  }

  
  public Command ManualRunIntake() {
    return runOnce(() -> intakeWheels.set(VictorSPXControlMode.PercentOutput, .5));
  }

  public Command ManualStopIntake() {
    return runOnce(() -> intakeWheels.set(VictorSPXControlMode.PercentOutput, 0));
  }

  @Logged
  public Command Intake(){
    return this.startEnd(
      () -> intakeWheels.set(VictorSPXControlMode.PercentOutput, .5), 
      () -> intakeWheels.set(VictorSPXControlMode.PercentOutput, 0)
    );
  }

  @Logged
  public Command Outtake(){
    return this.startEnd(
      () -> intakeWheels.set(VictorSPXControlMode.PercentOutput, -.5), 
      () -> intakeWheels.set(VictorSPXControlMode.PercentOutput, 0)
    );
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getArmSpeedDown(){
    double armSpeedDown = nt_armSpeedDown.getDouble(Constants.IntakeConstants.kIntakeArmSpeedDown);
    return armSpeedDown;
  }

  public double getArmSpeedUp(){
    double armSpeedUp = nt_armSpeedUp.getDouble(Constants.IntakeConstants.kIntakeArmSpeedUp);
    return armSpeedUp;
  }

  public void stopArm(){
    intakeWheels.set(VictorSPXControlMode.PercentOutput, 0);
  }


  private void setupShuffleboard(){
    ShuffleboardTab tab = Shuffleboard.getTab("Intake");

    nt_armSpeedDown = tab.addPersistent("Arm Speed Down", Constants.IntakeConstants.kIntakeArmSpeedDown)
    .withSize(3,1)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0, "max", 1))
    .getEntry();

    nt_armSpeedUp = tab.addPersistent("Arm Speed Up", Constants.IntakeConstants.kIntakeArmSpeedUp)
    .withSize(3,1)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0, "max", 1))
    .getEntry();   

    tab.add(intake_arm_pid);

    tab.add("ArmIn", ManualArmIn());
    tab.add("ArmOut", ManualArmOut());
    tab.add("Start Intake", ManualRunIntake());
    tab.add("Stop Intake", ManualStopIntake());
  }
}
