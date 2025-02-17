package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveXPark extends Command {

    private final SwerveSubsystem swerveSubsystem;

    double timeoutExpires;

    public SwerveXPark(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = {
            new SwerveModuleState(0, new Rotation2d(45)),
            new SwerveModuleState(0, new Rotation2d(225)),
            new SwerveModuleState(0, new Rotation2d(225)),
            new SwerveModuleState(0, new Rotation2d(45)),
        };

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}