package frc.robot.subsystems.blocker;

import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction layer for the Blocker.
 *
 * <p>The blocker is a single motor angle-controlled mechanism, so it uses closed-loop position
 * control just like the arm.
 */
public interface BlockerIO {

  @AutoLog
  public static class BlockerIOInputs {
    public boolean connected = false;
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  public static enum BlockerIOOutputMode {
    BRAKE,
    COAST,
    VOLTAGE,
    POSITION
  }

  public static class BlockerIOOutputs {
    public BlockerIOOutputMode mode = BlockerIOOutputMode.COAST;
    // Closed loop control
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double accelerationRadPerSec2 = 0.0;
    public double feedforwardAmps = 0.0;

    public double voltageOut = 0.0;
  }

  public default void updateInputs(BlockerIOInputs inputs) {}

  public default void applyOutputs(BlockerIOOutputs outputs) {}

  public default void resetAngle(double angleRads) {}

  public default void setPID(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}
}
