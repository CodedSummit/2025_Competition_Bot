package frc.robot;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(SparkMax.class)
public class SparkMaxLogger extends ClassSpecificLogger<SparkMax> {
  public SparkMaxLogger() {
    super(SparkMax.class);
  }

  @Override
  public void update(EpilogueBackend backend, SparkMax m){
    backend.log("Requested Speed (Duty Cycle)", m.get());
    backend.log("Voltage", m.getAppliedOutput());
    backend.log("Amps", m.getOutputCurrent());
    backend.log("Temperature", m.getMotorTemperature());
  }
}