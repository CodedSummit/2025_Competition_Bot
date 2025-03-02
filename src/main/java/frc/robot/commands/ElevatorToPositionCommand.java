// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ElevatorToPositionCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ElevatorSubsystem elevator;

  private double position_target;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorToPositionCommand(ElevatorSubsystem e, double p) {
    elevator = e;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(e);

    position_target = p;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double gap = position_target - elevator.getHeight();
    //check for already there

    //TODO: This doesn't work quite right
    if(gap > 0){
      elevator.ManualElevatorUp();
    } else {
      elevator.ManualElevatorDown();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // this is where we woud motion profile
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stopElevator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
