package frc.robot.subsystems.intake;

import frc.robot.Ports;

public class IntakeConstants {
  public static final int kIntakeId = Ports.kIntake;
  public static boolean kIntakeInverted = true;

  public static final double kIntakeRadius = 0.025; // TODO
  public static final double kIntakeGearRatio = 18.0 / 12.0; // 44.0 / 12.0; // 1.125

  public static final double kP = 50.0; // 10.0
  public static final double kI = 0.0;
  public static final double kD = 0.0; // 0.0;
  public static final double kG = 0.0; // 重力补偿
  public static final double kS = 0.1; // 静态前馈
  public static final double kV = 0.3; // 轨迹规划得到的目标速度
  public static final double kA = 0.0; // 轨迹规划得到的目标加速度

  public static final double kVelocityTolerance = 5.0;

  public static final double kIntakeVelocity = 400; // 180.0;
  public static final double kOuttakeVelocity = -400; // -180.0;
  public static final double kStowVelocity = 180.0;
  public static final double kShootVelocity = 100; // 50.0;

  public static final double kIntakeBPS = 30.0; // balls per second
}
