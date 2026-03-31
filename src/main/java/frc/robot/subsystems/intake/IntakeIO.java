package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean connected;
    public double positionRads;
    public double velocityRadsPerSec;
    public double appliedVoltage;
    public double supplyCurrentAmps;
    public double torqueCurrentAmps;
    public double tempCelsius;
  }

  public static enum IntakeIOOutputMode {
    COAST,
    VELOCITY,
    VOLTAGE
  }

  public static class IntakeIOOutputs {
    public IntakeIOOutputMode mode = IntakeIOOutputMode.VELOCITY;
    public double velocityRadsPerSec = 0.0;
    public double volts = 0.0;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  default void applyOutputs(IntakeIOOutputs outputs) {}

  /** Configure turret PID */
  public default void setPID(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}
}
