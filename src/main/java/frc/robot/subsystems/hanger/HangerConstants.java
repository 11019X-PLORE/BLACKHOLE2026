package frc.robot.subsystems.hanger;

import edu.wpi.first.math.util.Units;
import frc.robot.Ports;

public class HangerConstants {
  public static final int kHangerId = Ports.kHanger;
  public static final boolean kHangerInverted = true;

  public static final double kHangerRadius = 0.02; // TODO
  public static final double kHangerGearRatio = 67.5; // TODO

  public static final double kP = 10.0; // 100
  public static final double kI = 0.0;
  public static final double kD = 0.0; // 0.0;
  public static final double kG = 0.0; // 重力补偿
  public static final double kS = 0.1; // 静态前馈
  public static final double kV = 0.13; // 轨迹规划得到的目标速度
  public static final double kA = 0.2; // 轨迹规划得到的目标加速度

  public static final double kAccelerationRadPerSecSq = Units.degreesToRadians(500); // 最大角加速度
  public static final double kVelocityRadPerSec = Units.degreesToRadians(360);

  public static final double kHangerMinAngle = Units.degreesToRadians(0.0); // TODO:
  public static final double kHangerMaxAngle = Units.degreesToRadians(530.0);
  public static final double kHangerInitialAngle = Units.degreesToRadians(0.0);
  public static final double kHangerClimbAngle = Units.degreesToRadians(200.0);
  public static final double kExtended = Units.degreesToRadians(517.0);

  public static final double kHangeroffset =
      Units.degreesToRadians(0.0); // TODO: offset for hanger angle
  public static final double kHangertoleranceDeg = Units.degreesToRadians(2.0);

  public static final double kHangerStallVolts = -1.0; // 施加一个小的反向电压来检测堵转
  public static final double kHangerStallTorqueCurrent = 5.0;
  public static final double kHangerStallTime = 0.2;
  public static final double kHangerStallVelocity = Units.degreesToRadians(27.0); // 速度阈值，单位为弧度每秒
}
