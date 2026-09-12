package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction layer for the Flywheel.
 *
 * <p>On the offseason robot the flywheel acts as both the intake and the shooter, so it is a single
 * velocity-controlled motor that can spin either direction.
 */
public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public boolean connected = false;
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  public static enum FlywheelIOOutputMode {
    COAST,
    VELOCITY,
    VOLTAGE
  }

  public static class FlywheelIOOutputs {
    public FlywheelIOOutputMode mode = FlywheelIOOutputMode.COAST;
    public double velocityRadsPerSec = 0.0;
    public double accelerationRadPerSec2 = 0.0;
    public double feedforwardAmps = 0.0;
    public double volts = 0.0;
    public double num_motors = 3;
  }

  default void updateInputs(FlywheelIOInputs inputs) {}

  default void applyOutputs(FlywheelIOOutputs outputs) {}

  /** Configure flywheel PID */
  public default void setPID(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}
}
