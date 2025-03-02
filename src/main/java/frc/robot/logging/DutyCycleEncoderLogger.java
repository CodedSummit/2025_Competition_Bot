package frc.robot.logging;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

@CustomLoggerFor(DutyCycleEncoder.class)
public class DutyCycleEncoderLogger extends ClassSpecificLogger<DutyCycleEncoder> {
  public DutyCycleEncoderLogger() {
    super(DutyCycleEncoder.class);
  }

  @Override
  public void update(EpilogueBackend backend, DutyCycleEncoder e){
    backend.log("Absolute Rotation Angle Radians", e.get());
    backend.log("Channel", e.getSourceChannel());
  }
}