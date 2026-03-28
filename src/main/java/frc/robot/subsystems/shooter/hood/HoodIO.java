package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {

  @AutoLog
  public static class HoodIOInputs {
    // TODO: add encoder
    boolean motorConnected = false;
    double positionRads = 0.0;
    double velocityRadsPerSec = 0.0;
    double appliedVolts = 0.0;
    double supplyCurrentAmps = 0.0;
    double torqueCurrentAmps = 0.0;
    double tempCelsius = 0.0;
  }

  public static enum HoodIOOutputMode {
    BRAKE,
    COAST,
    CLOSED_LOOP
  }

  public static class HoodIOOutputs {

    public HoodIOOutputMode mode = HoodIOOutputMode.CLOSED_LOOP;
    // Closed loop control
    public double positionRad = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double kP = 0.0;
    public double kD = 0.0;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void applyOutputs(HoodIOOutputs outputs) {}

  public default void setPID(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}
}
