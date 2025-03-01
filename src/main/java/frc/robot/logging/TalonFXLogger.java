package frc.robot.logging;

import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(TalonFX.class)
public class TalonFXLogger extends ClassSpecificLogger<TalonFX> {
  public TalonFXLogger() {
    super(TalonFX.class);
  }

  @Override
  public void update(EpilogueBackend backend, TalonFX m){
    backend.log("Requested Speed (Duty Cycle)", m.get());
    backend.log("Voltage", m.getMotorVoltage().getValueAsDouble());
    backend.log("Stator Current", m.getStatorCurrent().getValueAsDouble());
    backend.log("Temperature", m.getDeviceTemp().getValueAsDouble());
  }
}