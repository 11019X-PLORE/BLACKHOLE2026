package frc.robot.subsystems.indexer;

import frc.robot.Ports;

public class IndexerConstants {
  public static final int kIndexerId = Ports.kIndexer;
  public static boolean kIndexerInverted = true;

  public static final double kIndexerRadius = 0.05; // TODO
  public static final double kIndexerGearRatio = 5.0; // 1.125

  public static final double kP = 100.0; // 10.0
  public static final double kI = 0.0;
  public static final double kD = 0.0; // 0.0;
  public static final double kG = 0.0; // 重力补偿
  public static final double kS = 0.1; // 静态前馈
  public static final double kV = 0.2; // 轨迹规划得到的目标速度
  public static final double kA = 0.0; // 轨迹规划得到的目标加速度

  public static final double kVelocityTolerance = 5.0;

  public static final double kIntakeVelocity = 125.0;
  public static final double kOuttakeVelocity = -125.0;
  public static final double kActiveVelocity = 125.0;
  public static final double kShootVelocity = 125.0;
}
