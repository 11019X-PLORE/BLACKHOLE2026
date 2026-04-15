package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.util.Units;
import frc.robot.Ports;

public class HoodConstants {
  public static final int kHoodId = Ports.kHood;
  public static final boolean kHoodInverted = true;

  public static final double kHoodRadius = 0.20193; // TODO
  public static final double kHoodGearRatio = 174.0 * 44.0 * 44.0 / (14.0 * 8.0 * 10.0); // TODO

  public static final double kP = 2000.0; // 500.0
  public static final double kI = 0.0;
  public static final double kD = 60.0; // 0.0;
  public static final double kG = 0.0; //
  public static final double kS = 0.6; // 0.6
  public static final double kV = 200; // 轨迹规划得到的目标速度
  public static final double kA = 0.0; // 轨迹规划得到的目标加速度

  public static final double kAccelerationRadPerSecSq = Units.degreesToRadians(180); // 最大角加速度
  public static final double kVelocityRadPerSec = Units.degreesToRadians(180);

  public static final double kHoodMinAngle = Units.degreesToRadians(13.703874); // TODO:
  public static final double kHoodMaxAngle = Units.degreesToRadians(38.703874);
  public static final double kHoodInitialAngle = Units.degreesToRadians(13.703874);
  public static final double kHoodPassingAngle = Units.degreesToRadians(32.0);
  public static final double kHoodTrenchAngle = Units.degreesToRadians(0.0);

  public static final double kInertiaHood = 0.125; // TODO
  public static final double kT = 0.01537;

  public static final double kHoodoffset =
      Units.degreesToRadians(0.0); // TODO: offset for hood angle
  public static final double kHoodtoleranceDeg = Units.degreesToRadians(10);

  public static final double kFixAngle = Units.degreesToRadians(20.0);

  public static final double kFixVelocity = Units.degreesToRadians(90);
}
