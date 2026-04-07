package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
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

  public static enum FlywheelIOOutputMode {
    COAST,
    VELOCITY,
    VOLTAGE,
    VELOCITY_FOC
  }

  public static class FlywheelIOOutputs {
    public FlywheelIOOutputMode mode = FlywheelIOOutputMode.VELOCITY;
    public double velocityRadsPerSec = 0.0;
    public double accelerationRadPerSec2 = 0.0;
    public double feedforwardAmps = 0.0;
    public double volts = 0.0;
  }

  default void updateInputs(FlywheelIOInputs inputs) {}

  default void applyOutputs(FlywheelIOOutputs outputs) {}

  /** Configure turret PID */
  public default void setPID(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}
}
