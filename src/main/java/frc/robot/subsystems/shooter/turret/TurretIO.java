package frc.robot.subsystems.shooter.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

  @AutoLog
  public static class TurretIOInputs {

    public boolean turretMotorConnected = false;
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double turretAlphaRadsPerSecSquared = 0.0;
    public double turretAppliedVolts = 0.0;
    public double turretSupplyCurrent = 0.0;
    public double turretTorqueCurrent = 0.0;
    public double turretTempCelsius = 0.0;
  }

  public static enum TurretIOOutputMode {
    BRAKE,
    COAST,
    CLOSED_LOOP,
    POSITION_FOC
  }

  public static class TurretIOOutputs {

    public TurretIOOutputMode mode = TurretIOOutputMode.BRAKE;
    // Closed loop control
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double accelerationRadPerSec2 = 0.0;
    public double feedforwardAmps = 0.0;
  }

  default void updateInputs(TurretIOInputs inputs) {}

  default void applyOutputs(TurretIOOutputs outputs) {}

  /** Configure turret PID */
  default void setPID(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  /** resetPosition */
  default void resetPosition(double angleRads) {}

  default void launchFuel() {}
}
