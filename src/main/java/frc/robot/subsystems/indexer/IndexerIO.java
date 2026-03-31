package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public boolean connected;
    public double positionRads;
    public double velocityRadsPerSec;
    public double appliedVoltage;
    public double supplyCurrentAmps;
    public double torqueCurrentAmps;
    public double tempCelsius;
  }

  public static enum IndexerIOOutputMode {
    COAST,
    VELOCITY,
    VOLTAGE
  }

  public static class IndexerIOOutputs {
    public IndexerIOOutputMode mode = IndexerIOOutputMode.VELOCITY;
    public double velocityRadsPerSec = 0.0;
    public double volts = 0.0;
  }

  default void updateInputs(IndexerIOInputs inputs) {}

  default void applyOutputs(IndexerIOOutputs outputs) {}

  /** Configure turret PID */
  public default void setPID(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}
}
