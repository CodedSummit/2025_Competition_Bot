package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SwerveSubsystem;

public class FinalAlignCommand extends Command {
    public SwerveSubsystem mSwerve;
    public final Pose2d goalPose;
    private PPHolonomicDriveController mDriveController = new PPHolonomicDriveController(
            new PIDConstants(2.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(2.0, 0.0, 0.0), // Rotation PID constants
            0.02 // Control loop duration in seconds
    );

    private final Trigger endTrigger;
    private final Trigger endTriggerDebounced;

    private final BooleanPublisher endTriggerLogger = NetworkTableInstance.getDefault().getTable("logging")
            .getBooleanTopic("PositionPIDEndTrigger").publish();

    public static final Time kEndTriggerDebounce = Seconds.of(0.04);
    public static final Distance kPositionTolerance = Inches.of(0.4);
    public static final Rotation2d kRotationTolerance = Rotation2d.fromDegrees(2.0);

    private FinalAlignCommand(SwerveSubsystem mSwerve, Pose2d goalPose) {
        this.mSwerve = mSwerve;
        this.goalPose = goalPose;

        endTrigger = new Trigger(() -> {
            Pose2d diff = mSwerve.getPose().relativeTo(goalPose);

            var rotation = MathUtil.isNear(
                    0.0,
                    diff.getRotation().getRotations(),
                    kRotationTolerance.getRotations(),
                    0.0,
                    1.0);

            var position = diff.getTranslation().getNorm() < kPositionTolerance.in(Meters);
            System.out.println("end trigger conditions R: " + rotation + "\tP: " + position);
            return rotation && position;
        });

        endTriggerDebounced = endTrigger.debounce(kEndTriggerDebounce.in(Seconds));
    }

    public static Command generateCommand(SwerveSubsystem swerve, Pose2d goalPose, Time timeout) {
        return new FinalAlignCommand(swerve, goalPose).withTimeout(timeout).finallyDo(() -> {
            swerve.driveRobotRelative(new ChassisSpeeds(0, 0, 0), null);
        });
    }

    @Override
    public void initialize() {
        endTriggerLogger.accept(endTrigger.getAsBoolean());
    }

    @Override
    public void execute() {
        System.out.println("  Execute FinalAlignCommand");
        PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
        goalState.pose = goalPose;

        endTriggerLogger.accept(endTrigger.getAsBoolean());

        mSwerve.driveRobotRelative(
                mDriveController.calculateRobotRelativeSpeeds(
                        mSwerve.getPose(), goalState),
                null);
    }

    @Override
    public void end(boolean interrupted) {
        endTriggerLogger.accept(endTrigger.getAsBoolean());
    }

    @Override
    public boolean isFinished() {
        return endTriggerDebounced.getAsBoolean();
    }
}
