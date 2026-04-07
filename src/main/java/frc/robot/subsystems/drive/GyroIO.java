package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public Rotation2d yawPosition = Rotation2d.kZero;
    public double yawVelocityRadPerSec = 0.0;
    public double[] odometryYawTimestamps = new double[] {};
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    public Rotation2d[] odometryPitchPositions = new Rotation2d[] {};
    public Rotation2d[] odometryRollPositions = new Rotation2d[] {};
    public double[] odometryYawVelocities = new double[] {};
    public double[] odometryAccelX = new double[] {};
    public double[] odometryAccelY = new double[] {};
    public double[] odometryAccelZ = new double[] {};
  }

  public default void updateInputs(GyroIOInputs inputs) {}
}
