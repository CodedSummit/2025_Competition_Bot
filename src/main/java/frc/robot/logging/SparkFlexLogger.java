package frc.robot.logging;

import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(SparkFlex.class)
public class SparkFlexLogger extends ClassSpecificLogger<SparkFlex> {
  public SparkFlexLogger() {
    super(SparkFlex.class);
  }

  @Override
  public void update(EpilogueBackend backend, SparkFlex m){
    backend.log("Requested Speed (Duty Cycle)", m.get());
    backend.log("Voltage", m.getAppliedOutput());
    backend.log("Amps", m.getOutputCurrent());
    backend.log("Temperature", m.getMotorTemperature());
    backend.log("ForwardLimitSwitch", m.getForwardLimitSwitch().isPressed());
    backend.log("ReverseLimitSwitch", m.getReverseLimitSwitch().isPressed());
    
  }
}