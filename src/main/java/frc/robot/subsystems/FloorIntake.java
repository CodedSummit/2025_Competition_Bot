// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SetWheelAlignment;
import frc.robot.commands.ZeroOdometry;

@Logged
public class FloorIntake extends SubsystemBase {

  private final VictorSPX intakeArmMotor = new VictorSPX(11);
  private final VictorSPX intakeWheels = new VictorSPX(10);

  private final DutyCycleEncoder intakeArmPosition = new DutyCycleEncoder(9);


  
  /** Creates a new FloorIntake. */
  public FloorIntake() {
    
    intakeWheels.setInverted(true);
    this.initialize();
  }


    public void initialize() {

        ShuffleboardTab tab = Shuffleboard.getTab("Intake");


        tab.add("ArmUp", ManualArmDown());
        tab.add("ArmDown", ManualArmUp());
        tab.add("Start Intake", ManualRunIntake());
        tab.add("Stop Intake", ManualStopIntake());


    }


  public double armPosition(){
    return intakeArmPosition.get();
  }

  @Logged
  public Command ManualArmDown(){
    return this.startEnd(
      () -> intakeArmMotor.set(VictorSPXControlMode.PercentOutput, .4), 
      () -> intakeArmMotor.set(VictorSPXControlMode.PercentOutput, 0)
    );
  }

  @Logged
  public Command ManualArmUp(){
    return this.startEnd(
      () -> intakeArmMotor.set(VictorSPXControlMode.PercentOutput, -.4), 
      () -> intakeArmMotor.set(VictorSPXControlMode.PercentOutput, 0)
    );
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


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
