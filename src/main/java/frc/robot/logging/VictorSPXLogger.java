package frc.robot.logging;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(VictorSPX.class)
public class VictorSPXLogger extends ClassSpecificLogger<VictorSPX> {
  public VictorSPXLogger() {
    super(VictorSPX.class);
  }

  @Override
  public void update(EpilogueBackend backend, VictorSPX m){
    backend.log("Requested Speed (Percentage)", m.getMotorOutputPercent());
    backend.log("Voltage", m.getMotorOutputVoltage());
    backend.log("Temperature", m.getTemperature());
  }
}