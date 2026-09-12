package frc.robot.subsystems.flywheel;

import frc.robot.Ports;

public class FlywheelConstants {
  public static int[] kFlywheelIds = {Ports.kFlywheel0, Ports.kFlywheel1, Ports.kFlywheel2};
  public static boolean kFlywheelInverted = true;

  public static final double kFlywheelRadius = 0.055; // TODO
  public static final double kFlywheelGearRatio = 1.0;

  public static final double kP = 0.5;
  public static final double kI = 0.05;
  public static final double kD = 0.0;
  public static final double kG = 0.0;
  public static final double kS = 0;
  public static final double kV = 0.13;
  public static final double kA = 0.0;

  public static final double kVelocityTolerance = 15.0; // rad/s

  // --- Setpoints (rad/s) ---
  public static final double kIntakeVelocity = 250.0; // spin in to intake
  public static final double kShootVelocity = -300.0; // spin fast to shoot
  public static final double kOuttakeVelocity = -300.0; // reverse to spit out

  public static final double stuckThresholdVelocity = 30;
  public static final double stuckThresholdTimeS = 0.2;
  public static final double backSpinTime = 0.2;
}
