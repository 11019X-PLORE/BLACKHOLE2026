package frc.robot.subsystems.intakearm;

import org.littletonrobotics.junction.AutoLog;

public interface IntakearmIO {

  @AutoLog
  public static class IntakearmIOInputs {
    // TODO: add encoder
    boolean motorConnected = false;
    double positionRads = 0.0;
    double velocityRadsPerSec = 0.0;
    double appliedVolts = 0.0;
    double supplyCurrentAmps = 0.0;
    double torqueCurrentAmps = 0.0;
    double tempCelsius = 0.0;
  }

  public static enum IntakearmIOOutputMode {
    BRAKE,
    COAST,
    CLOSED_LOOP,
    VOLTAGE
  }

  public static class IntakearmIOOutputs {
    public IntakearmIOOutputMode mode = IntakearmIOOutputMode.CLOSED_LOOP;
    // Closed loop control
    public double positionRad = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double kP = 0.0;
    public double kD = 0.0;
    // Voltage control
    public double appliedVolts = 0.0;
  }

  public default void updateInputs(IntakearmIOInputs inputs) {}

  public default void applyOutputs(IntakearmIOOutputs outputs) {}

  public default void setPID(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  public default void resetPosition(double angleRads) {}
}
