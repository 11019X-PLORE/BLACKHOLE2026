package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction layer for the Arm.
 *
 * <p>The arm both deploys the intake and controls the shooting angle, so it is a single jointed arm
 * with closed-loop position control.
 */
public interface ArmIO {

  @AutoLog
  public static class ArmIOInputs {
    public boolean connected = false;
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  public static enum ArmIOOutputMode {
    BRAKE,
    COAST,
    POSITION
  }

  public static class ArmIOOutputs {
    public ArmIOOutputMode mode = ArmIOOutputMode.COAST;
    // Closed loop control
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double accelerationRadPerSec2 = 0.0;
    public double feedforwardAmps = 0.0;
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void applyOutputs(ArmIOOutputs outputs) {}

  public default void resetAngle(double angleRads) {}

  public default void setPID(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}
}
