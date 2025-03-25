package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import java.util.Map;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.SetWheelAlignment;
import frc.robot.commands.ZeroOdometry;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.RobotConfig;

@Logged
public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.FrontLeft.DriveMotor,
            DriveConstants.FrontLeft.TurningMotor,
            DriveConstants.FrontLeft.DriveEncoderReversed,
            DriveConstants.FrontLeft.TurningEncoderReversed,
            DriveConstants.FrontLeft.TurningAbsoluteEncoder,
            DriveConstants.FrontLeft.DriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.FrontRight.DriveMotor,
            DriveConstants.FrontRight.TurningMotor,
            DriveConstants.FrontRight.DriveEncoderReversed,
            DriveConstants.FrontRight.TurningEncoderReversed,
            DriveConstants.FrontRight.TurningAbsoluteEncoder,
            DriveConstants.FrontRight.DriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.BackLeft.DriveMotor,
            DriveConstants.BackLeft.TurningMotor,
            DriveConstants.BackLeft.DriveEncoderReversed,
            DriveConstants.BackLeft.TurningEncoderReversed,
            DriveConstants.BackLeft.TurningAbsoluteEncoder,
            DriveConstants.BackLeft.DriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.BackRight.DriveMotor,
            DriveConstants.BackRight.TurningMotor,
            DriveConstants.BackRight.DriveEncoderReversed,
            DriveConstants.BackRight.TurningEncoderReversed,
            DriveConstants.BackRight.TurningAbsoluteEncoder,
            DriveConstants.BackRight.DriveAbsoluteEncoderReversed);

    //private final ADIS16470_IMU gyro = new ADIS16470_IMU();
    Pigeon2 gyro = new Pigeon2(20); // Pigeon is on CAN Bus with device ID 0

    private final SwerveDrivePoseEstimator odometer = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
            new Rotation2d(0), getModulePositions(), new Pose2d());

    private VisionPoseEstimationSubsystem m_VisionPoseEstimationSubsystem;

    private GenericEntry turboSpeedFactor;
    private GenericEntry normalSpeedFactor;
    private GenericEntry dampenedSpeedFactor;

    private Rotation2d offsetYawAngle = new Rotation2d(0);

    public RobotConfig config;

    public boolean calibratedDirection = false;

    @Logged
    public final Field2d m_field = new Field2d();

    public SwerveSubsystem() {
        this.initialize();
        Matrix<N3, N1> visionMeasurementStdDevs =  new Matrix<>(Nat.N3(), Nat.N1());
        visionMeasurementStdDevs.set(0,0,Constants.VisionConstants.kVisionXStdDev);
        visionMeasurementStdDevs.set(1,0,Constants.VisionConstants.kVisionYStdDev);
        visionMeasurementStdDevs.set(2,0,Constants.VisionConstants.kVisionThetaStdDev);

        odometer.setVisionMeasurementStdDevs(visionMeasurementStdDevs);
    }

    public SwerveSubsystem( VisionPoseEstimationSubsystem vPoseEstimation){
        m_VisionPoseEstimationSubsystem = vPoseEstimation;
        this.initialize();
    }

    public void initialize() {

        loadPreferences();

        ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve");
        swerveTab.add("Front Left", frontLeft)
                .withSize(2, 2);
        swerveTab.add("Front Right", frontRight)
                .withSize(2, 2);
        swerveTab.add("Back Right", backRight)
                .withSize(2, 2)
                .withPosition(0, 2);
        swerveTab.add("Back Left", backLeft)
                .withSize(2, 2)
                .withPosition(2, 2);
        swerveTab.add("FL PID", frontLeft.turningPidController);

        swerveTab.add("Set Wheel Offsets", new SetWheelAlignment(this));

        turboSpeedFactor = swerveTab.add("Turbo Percentage", 0.9)
                .withWidget(BuiltInWidgets.kNumberSlider) // specify the widget here
                .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
                .getEntry();

        normalSpeedFactor = swerveTab.add("Normal Percentage", .5)
                .withWidget(BuiltInWidgets.kNumberSlider) // specify the widget here
                .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
                .getEntry();

        dampenedSpeedFactor = swerveTab.add("Dampened Percentage", .2)
                .withWidget(BuiltInWidgets.kNumberSlider) // specify the widget here
                .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
                .getEntry();

        swerveTab.addDouble("Gyro data", () -> gyro.getYaw().getValueAsDouble());

        swerveTab.add("Field", m_field);

        swerveTab.add(new ZeroOdometry(this));

        SmartDashboard.putData("Swerve Drive", new Sendable() {
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("SwerveDrive");

    builder.addDoubleProperty("Front Left Angle", () -> frontLeft.getTurningPosition(), null);
    builder.addDoubleProperty("Front Left Velocity", () -> frontLeft.getDriveVelocity(), null);

    builder.addDoubleProperty("Front Right Angle", () -> frontRight.getTurningPosition(), null);
    builder.addDoubleProperty("Front Right Velocity", () -> frontRight.getDriveVelocity(), null);

    builder.addDoubleProperty("Back Left Angle", () -> backLeft.getTurningPosition(), null);
    builder.addDoubleProperty("Back Left Velocity", () -> backLeft.getDriveVelocity(), null);

    builder.addDoubleProperty("Back Right Angle", () -> backRight.getTurningPosition(), null);
    builder.addDoubleProperty("Back Right Velocity", () -> backRight.getDriveVelocity(), null);

    builder.addDoubleProperty("Robot Angle", () -> getRotation2d().getRadians(), null);
  }
});

        // Configure AutoBuilder last
        try {
            config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                    this::getPose, // Robot pose supplier
                    this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                    this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                    new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your
                                                    // Constants class
                            new PIDConstants(2.0, 0.0, 0.0), // Translation PID constants
                            new PIDConstants(2.0, 0.0, 0.0), // Rotation PID constants
                            0.02 // Control loop length in seconds
                    ),
                    config,

                    () -> {
                        var alliance = DriverStation.getAlliance(); // Boolean supplier that checks which alliance the
                                                                    // bot is on
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this);

        } catch (Exception e) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
        }

    }

    public double getTurboSpeedFactor(){
        return turboSpeedFactor.getDouble(0.5);
    }
    public double getNormalSpeedFactor(){
        return normalSpeedFactor.getDouble(0.3);
    }
    public double getDampenedSpeedFactor(){
        return dampenedSpeedFactor.getDouble(0.1);
    }

    public void lockEncoderOffset(){
        frontLeft.lockEncoderOffset();
        frontRight.lockEncoderOffset();
        backLeft.lockEncoderOffset();
        backRight.lockEncoderOffset();
    }

    public void loadPreferences(){
        frontLeft.loadPreferences();
        frontRight.loadPreferences();
        backLeft.loadPreferences();
        backRight.loadPreferences();
    }

    public Command zeroHeadingCommand(){
        return this.runOnce(() -> zeroHeading());
    }

    public void zeroHeading() {
        //gyro.reset();
        offsetYawAngle = new Rotation2d(gyro.getYaw().getValue());
        calibratedDirection = true;
        
        //gyro.setYaw(0);
    }

    public double DriverOffsetYaw(){
        return offsetYawAngle.getDegrees();
    }

    public double OdometerRotation(){
        return odometer.getEstimatedPosition().getRotation().getDegrees();
    }

    public double CalcGyroAngle(){
        return gyro.getYaw().getValueAsDouble();
    }


    public void zeroHeadingWithVision(boolean isRed){
        Rotation2d odometer_rotation = odometer.getEstimatedPosition().getRotation();
        Rotation2d gyroAngle = new Rotation2d(gyro.getYaw().getValue());

        offsetYawAngle = gyroAngle.minus(odometer_rotation);

        //if red alliance, flip 180
        if(isRed){
            offsetYawAngle = offsetYawAngle.rotateBy(Rotation2d.k180deg);
        }
        calibratedDirection = true;
    }

    public void setHeading(double setAngle){
        gyro.setYaw(setAngle);
    }

    public double getHeading() {
        //return Math.IEEEremainder(gyro.getAngle(gyro.getYawAxis()), 360); 
        return Math.IEEEremainder(gyro.getYaw().getValueAsDouble(), 360.0);   
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Rotation2d getOffsetRotation2d(){
        return getRotation2d().minus(offsetYawAngle);
    }

    public Pose2d getPose() {
        return odometer.getEstimatedPosition();
    }
    public void resetPose(Pose2d pose2d1) {
        resetOdometry(pose2d1);
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble()), getModulePositions(), pose);
        //gyro.setGyroAngleZ(pose.getRotation().getDegrees());
        //gyro.setYaw(pose.getRotation().getDegrees());
    }
    public void resetOdometry(){
        resetOdometry(new Pose2d());
    }

    private SwerveModulePosition [] getModulePositions(){
        return new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(),
            backRight.getPosition()};
    }

    @Override
    public void periodic() {
        
        updatePose();
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        m_field.setRobotPose(getPose());
        SmartDashboard.putNumber("PoseX", getPose().getX() );
        SmartDashboard.putNumber("PoseY", getPose().getY() );
    }

    

    public void updatePose(){
        m_VisionPoseEstimationSubsystem.updatePoseWithVision(odometer);
        odometer.update(getRotation2d(), getModulePositions());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState [] statelist = {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        };
        return statelist;
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        // TODO Auto-generated method stub
        super.initSendable(builder);

    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());

    }

    public void driveRobotRelative(ChassisSpeeds chassisspeeds1, DriveFeedforwards forwards) {
         // 5. Convert chassis speeds to individual module states
         SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisspeeds1);

         // 6. Output each module states to wheels
         setModuleStates(moduleStates);
    }

    public Command followPathCommand(String pathName) {
        PathPlannerPath path = null;
        Command command = null;
        config = null;
        try {
            path = PathPlannerPath.fromPathFile(pathName);
        } catch (Exception e) {
            DriverStation.reportError("Failed to load PathPlanner file:" + pathName, e.getStackTrace());
        }

        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            DriverStation.reportError("Failed to load RobotConfig settings from GUI", null);
        }
  //      System.out.println("  Generating path for file:" + pathName);
        command = new FollowPathCommand(
                path,
                this::getPose, // Robot pose supplier
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your
                                                // Constants class
                        new PIDConstants(2.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(2.0, 0.0, 0.0), // Rotation PID constants
                        0.02 // Control loop duration in seconds
                ),
                config,
                () -> {
                    var alliance = DriverStation.getAlliance(); // Boolean supplier that checks which alliance the bot
                                                                // is on
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );

        return command;
    }

}