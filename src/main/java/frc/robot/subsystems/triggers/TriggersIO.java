package frc.robot.subsystems.triggers;

import org.littletonrobotics.junction.AutoLog;

public interface TriggersIO {
  @AutoLog
  public static class TriggersIOInputs {
    public boolean connected;
    public double positionRads;
    public double velocityRadsPerSec;
    public double appliedVoltage;
    public double supplyCurrentAmps;
    public double torqueCurrentAmps;
    public double tempCelsius;

    public boolean secondconnected;
    public double secondpositionRads;
    public double secondvelocityRadsPerSec;
    public double secondappliedVoltage;
    public double secondsupplyCurrentAmps;
    public double secondtorqueCurrentAmps;
    public double secondtempCelsius;
  }

  public static enum TriggersIOOutputMode {
    COAST,
    VELOCITY,
    VOLTAGE
  }

  public static class TriggersIOOutputs {
    public TriggersIOOutputMode mode = TriggersIOOutputMode.VELOCITY;
    public double velocityRadsPerSec = 0.0;
    public double volts = 0.0;
  }

  default void updateInputs(TriggersIOInputs inputs) {}

  default void applyOutputs(TriggersIOOutputs outputs) {}

  /** Configure turret PID */
  public default void setPID(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}
}
