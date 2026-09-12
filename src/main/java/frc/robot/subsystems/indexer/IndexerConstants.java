package frc.robot.subsystems.indexer;

import frc.robot.Ports;

public class IndexerConstants {
  public static final int kIndexerId = Ports.kIndexer;
  public static boolean kIndexerInverted = false;

  public static final int kTriggerId = Ports.kTrigger;
  public static boolean kTriggerInverted = true;

  public static final double kIndexerRadius = 0.018; // TODO
  public static final double kIndexerGearRatio = 1.0;

  public static final double kIndexerSupplyCurrentLimit = 40.0;

  public static final double kP = 0.50;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kS = 0.1;
  public static final double kV = 0.2;
  public static final double kA = 0.0;

  public static final double triggerKP = 0.2;
  public static final double triggerKI = 0.0;
  public static final double triggerKD = 0.0;
  public static final double triggerKS = 0.1;
  public static final double triggerKV = 0.2;
  public static final double triggerKA = 0.0;

  public static final double kVelocityTolerance = 5.0; // rad/s

  // --- Setpoints (rad/s) ---
  public static final double kIntakeVelocity = 30.0; // pull the ball in
  public static final double kShootVelocity = -400.0; // feed the ball to the flywheel
  public static final double kOuttakeVelocity = -125.0; // reverse the ball out

  public static final double kIntakeTriggerVelocity = 250.0;
  public static final double kShootTriggerVelocity = -500.0; // feed the ball to the flywheel
  public static final double kOuttakeTriggerVelocity = -300.0; // reverse the ball out
}
