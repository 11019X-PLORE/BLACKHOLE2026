package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.util.Units;
import frc.robot.Ports;

public class HoodConstants {
  public static final int kHoodId = Ports.kHood;
  public static final boolean kHoodInverted = true;

  public static final double kHoodRadius = 0.20193; // TODO
  public static final double hoodGearRatio = 26.5; // TODO

  public static final double kP = 200.0; // 100
  public static final double kI = 0.0;
  public static final double kD = 0.0; // 0.0;
  public static final double kG = 0.0; // 重力补偿
  public static final double kS = 0.1; // 静态前馈
  public static final double kV = 0.13; // 轨迹规划得到的目标速度
  public static final double kA = 0; // 轨迹规划得到的目标加速度

  public static final double kAccelerationRadPerSecSq = Units.degreesToRadians(180); // 最大角加速度
  public static final double kVelocityRadPerSec = Units.degreesToRadians(180);

  public static final double kHoodMinAngle = Units.degreesToRadians(0.0); // TODO:
  public static final double kHoodMaxAngle = Units.degreesToRadians(25.0);
  public static final double kHoodInitialAngle = Units.degreesToRadians(0.0);
  public static final double kHoodPassingAngle = Units.degreesToRadians(25.0);
  public static final double kHoodTrenchAngle = Units.degreesToRadians(0.0);

  public static final double kHoodoffset =
      Units.degreesToRadians(0.0); // TODO: offset for hood angle
  public static final double kHoodtoleranceDeg = Units.degreesToRadians(1.0);

  public static final double kFixAngle = Units.degreesToRadians(5.72);

  public static final double kFixVelocity = Units.degreesToRadians(90);
}
