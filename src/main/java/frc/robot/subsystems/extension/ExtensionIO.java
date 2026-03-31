package frc.robot.subsystems.extension;

import org.littletonrobotics.junction.AutoLog;

public interface ExtensionIO {

  @AutoLog
  public static class ExtensionIOInputs {
    // TODO: add encoder
    boolean motorConnected = false;
    double positionRads = 0.0;
    double velocityRadsPerSec = 0.0;
    double appliedVolts = 0.0;
    double supplyCurrentAmps = 0.0;
    double torqueCurrentAmps = 0.0;
    double tempCelsius = 0.0;
  }

  public static enum ExtensionIOOutputMode {
    BRAKE,
    COAST,
    CLOSED_LOOP,
    VOLTAGE
  }

  public static class ExtensionIOOutputs {
    public ExtensionIOOutputMode mode = ExtensionIOOutputMode.CLOSED_LOOP;
    // Closed loop control
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double kP = 0.0;
    public double kD = 0.0;
    // Voltage control
    public double appliedVolts = 0.0;
  }

  public default void updateInputs(ExtensionIOInputs inputs) {}

  public default void applyOutputs(ExtensionIOOutputs outputs) {}

  public default void setPID(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  public default void resetPosition(double angleRads) {}
}
