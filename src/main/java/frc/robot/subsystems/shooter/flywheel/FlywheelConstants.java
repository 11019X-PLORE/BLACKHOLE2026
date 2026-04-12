package frc.robot.subsystems.shooter.flywheel;

import frc.robot.Ports;

public class FlywheelConstants {
  public static final int kFlywheelId = Ports.kFlywheel;
  public static final int kSecondFlywheel = Ports.kSecondFlywheel;
  public static boolean kFlywheelInverted = true;

  public static final double kFlywheelRadius = 0.05; // TODO
  public static final double kFlywheelGearRatio = 1.0; // 1.125

  public static final double kP = 100.0; // 10.0
  public static final double kI = 2;
  public static final double kD = 6; // 0.0;
  public static final double kG = 0.0; // 重力补偿
  public static final double kS = 0.1; // 静态前馈
  public static final double kV = 0.3; // 轨迹规划得到的目标速度
  public static final double kA = 0.0; // 轨迹规划得到的目标加速度

  public static final double kVelocityTolerance = 15.0;

  public static final double kAccelerationRadPerSecSq = 180.0; // 最大角加速度
  public static final double kVelocityRadPerSec = 180.0;

  public static final double kFixVelocity = 400.0;
  public static final double kPassVelocity = 400.0;
  public static final double kOutTakeVelocity = -400.0;
  public static final double kActiveRatio = 0.6;
}
