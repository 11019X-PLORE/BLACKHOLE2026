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

    public boolean indexRightconnected;
    public double indexRightvelocityRadsPerSec;

    // Triggers inputs (merged from TriggersIO)
    public boolean triggersConnected;
    public double triggersVelocityRadsPerSec;

    public boolean triggersRightconnected;
    public double triggersRightVelocityRadsPerSec;

    public boolean feedingLeft;
    public boolean leftLimitSwitchconnected;
    public boolean rightLimitSwitchconnected;
    public boolean leftLimitSwitch;
    public boolean rightLimitSwitch;
    public double triggerDistance;
  }

  public static enum IndexerIOOutputMode {
    COAST,
    VELOCITY,
    FEEDING,
  }

  public static class IndexerIOOutputs {
    public IndexerIOOutputMode mode = IndexerIOOutputMode.VELOCITY;
    public double velocityRadsPerSec = 0.0;
    public double maxVelocityRadsPerSec = 0.0;
    public double maxCurrent = 0.0;

    // Triggers outputs (merged from TriggersIO)
    public IndexerIOOutputMode triggersMode = IndexerIOOutputMode.VELOCITY;
    public double triggersVelocityRadsPerSec = 0.0;

    public boolean forceFeedLeft = false;
    public boolean forceFeedRight = false;
  }

  default void updateInputs(IndexerIOInputs inputs) {}

  default void applyOutputs(IndexerIOOutputs outputs) {}

  /** Configure indexer PID */
  public default void setPID(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  /** Configure triggers PID */
  public default void setTriggersPID(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}
}
