package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class SwerveModule implements Sendable {
    private final TalonFX driveTalonFX;
    private final SparkMax turningMotor;

    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    private final DutyCycleEncoder directionDutyCycle;

    private final boolean absoluteEncoderReversed;
    private double absoluteEncoderOffsetRad;

    private final int driveid;
    private final String encoderOffsetKey;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, boolean absoluteEncoderReversed) { // removed args: double absoluteEncoderOffset

        driveid = driveMotorId;

        // we load the absolute encoder offsets from config, allowing easier calibration.
        encoderOffsetKey = "absoluteEndcoderOffsetRadWheel"+driveid;
        Preferences.initDouble(encoderOffsetKey, 0);

        this.absoluteEncoderReversed = absoluteEncoderReversed;

        driveTalonFX = new TalonFX(driveMotorId, "rio");
        turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);

        
        directionDutyCycle = new DutyCycleEncoder(absoluteEncoderId);
        directionDutyCycle.setDutyCycleRange(1/4096, 4096/4096);

        driveTalonFX.getConfigurator().apply(new TalonFXConfiguration());

        driveTalonFX.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        turningEncoder = turningMotor.getEncoder();

//        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
//       turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.setTolerance(0.0046*10);
        turningPidController.enableContinuousInput(0, 2 * Math.PI);
        loadPreferences(); //to set initial values from storage
        resetEncoders();
    }

    public void lockEncoderOffset(){
        Preferences.setDouble(encoderOffsetKey, getRawAbsoluteEncoderRad());
        loadPreferences(); //to read it back out via round trip.
    }

    public void loadPreferences(){
        this.absoluteEncoderOffsetRad = Preferences.getDouble(encoderOffsetKey, 0);
    }

    public double getDrivePosition() {
        return (driveTalonFX.getRotorPosition().getValue()).magnitude() * ModuleConstants.kDriveEncoderRot2Meter;
    }

    public double getTurningPosition() {
        SmartDashboard.putNumber("Wheel-"+driveid, getAbsoluteEncoderRad());
        return getAbsoluteEncoderRad();
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            this.getDrivePosition(), new Rotation2d(this.getTurningPosition()));
      }

    public double getDriveVelocity() {
        return (driveTalonFX.get()) 
         * ModuleConstants.kDriveEncoderRPM2MeterPerSec;
    }
    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public double getRawAbsoluteEncoderRad(){
        // returns the encoder value without applying the configured offset
        double angle = directionDutyCycle.get();
        
        angle *= 2.0 * Math.PI; //convert 0-1 range into radians

        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);        
    }

    public double getAbsoluteEncoderRad() {
        double angle = directionDutyCycle.get();
        
        angle *= 2.0 * Math.PI; //convert 0-1 range into radians

        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveTalonFX.setPosition(0.0);
        turningEncoder.setPosition(getAbsoluteEncoderRad()); //TODO: this isn't referencing the right encoder. now using directionDutyCycle
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getState().angle);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));

        /*if (Math.abs(state.speedMetersPerSecond) < 0.005) {
            //driveTalonFX.set(ControlMode.PercentOutput, 0);
            driveTalonFX.set(0);
            return;
        }*/
        driveTalonFX.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    }

    public void stop() {
        driveTalonFX.set(0);
        turningMotor.set(0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // These are the values placed on the dashboard if the module is added.
        builder.addDoubleProperty("Drive Percentage", () -> driveTalonFX.getDutyCycle().getValue() , null);
        builder.addDoubleProperty("Rotate Percentage", () -> turningMotor.getAppliedOutput() , null);
        builder.addDoubleProperty("RotationRad", () -> getTurningPosition(), null);
    }
}