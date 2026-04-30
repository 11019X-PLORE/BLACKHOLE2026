package frc.robot.subsystems.indexer;

import frc.robot.Ports;

public class IndexerConstants {
  // === Indexer Motor ===
  public static final int kIndexerLeftId = Ports.kIndexerLeft;
  public static final int kIndexerRightId = Ports.kIndexerRight;

  public static boolean kIndexerLeftInverted = false;
  public static boolean kIndexerRightInverted = !kIndexerLeftInverted;

  public static final double kIndexerRadius = 0.05; // TODO
  public static final double kIndexerGearRatio = 3.0;

  public static final double kIndexerStatorCurrentLimit = 50;

  public static final double kP = 100.0; // 10.0
  public static final double kI = 0.0;
  public static final double kD = 0.0; // 0.0;
  public static final double kG = 0.0; // 重力补偿
  public static final double kS = 0.1; // 静态前馈
  public static final double kV = 0.2; // 轨迹规划得到的目标速度
  public static final double kA = 0.0; // 轨迹规划得到的目标加速度

  public static final double kVelocityTolerance = 5.0;

  public static final double kIntakeVelocity = 80.0;
  public static final double kOuttakeVelocity = -125.0;
  public static final double kActiveVelocity = 80.0;
  public static final double kShootVelocity = 200.0;

  // === Triggers Motors (merged from TriggersConstants) ===
  public static final int kTriggersLeftId = Ports.kTestLeftTriggers;
  public static final int kTriggersRightId = Ports.kTestRightTriggers;
  public static final int kTriggersLeftLimitSwitchId = Ports.kTestLeftLimitSwitch;
  public static final int kTriggersRightLimitSwitchId = Ports.kTestRightLimitSwitch;

  public static boolean kTriggersLeftInverted = false;
  public static boolean kTriggersRightInverted = true;

  public static double kTriggersMinSwitchPeriod = 0.2;

  public static final double kTriggersRadius = 0.05; // TODO
  public static final double kTriggersGearRatio = 5.0; // 1.125

  public static final double kTriggers_kP = 50.0;
  public static final double kTriggers_kI = 0.0;
  public static final double kTriggers_kD = 0.0;
  public static final double kTriggers_kG = 0.0;
  public static final double kTriggers_kS = 0.1;
  public static final double kTriggers_kV = 0.0;
  public static final double kTriggers_kA = 0.0;

  public static final double kTriggersVelocityTolerance = 5.0;

  public static final double kTriggersIntakeVelocity = 150.0;
  public static final double kTriggersOuttakeVelocity = -150.0;
  public static final double kTriggersShootVelocity = 150.0;

  public static final double feedingBPS = 7.5; // balls per second
}
