package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollerSystemIO {
  @AutoLog
  public static class RollerSystemIOInputs {
    public boolean connected;
    public double positionRads;
    public double velocityRadsPerSec;
    public double appliedVoltage;
    public double supplyCurrentAmps;
    public double torqueCurrentAmps;
    public double tempCelsius;
  }

  public static class RollerSystemIOOutputs {
    public double appliedVoltage = 0.0;
    public boolean brakeModeEnabled = true;
  }

  default void updateInputs(RollerSystemIOInputs inputs) {}

  default void applyOutputs(RollerSystemIOOutputs outputs) {}
}
