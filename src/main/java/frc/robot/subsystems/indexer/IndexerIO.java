package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction layer for the Indexer.
 *
 * <p>The offseason indexer is a single roller motor that indexes the ball, so it is a single
 * velocity-controlled motor.
 */
public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public boolean connected = false;
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  public static enum IndexerIOOutputMode {
    COAST,
    VELOCITY
  }

  public static class IndexerIOOutputs {
    public IndexerIOOutputMode mode = IndexerIOOutputMode.COAST;
    public double indexerVelocityRadsPerSec = 0.0;
    public double triggerVelocityRadsPerSec = 0.0;
  }

  default void updateInputs(IndexerIOInputs inputs) {}

  default void applyOutputs(IndexerIOOutputs outputs) {}

  /** Configure indexer PID */
  public default void setIndexerPID(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  public default void setTriggerPID(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}
}
