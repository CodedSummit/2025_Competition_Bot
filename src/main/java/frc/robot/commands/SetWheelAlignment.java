package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class SetWheelAlignment extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private boolean done = true;

    public SetWheelAlignment(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }


    @Override
    public void initialize() {
        done = false;
    }

    @Override
    public void execute() {
        swerveSubsystem.lockEncoderOffset();
        done = true;
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public boolean runsWhenDisabled() {
        // This is a config command, so must run disabled.
        return true;
    }
}