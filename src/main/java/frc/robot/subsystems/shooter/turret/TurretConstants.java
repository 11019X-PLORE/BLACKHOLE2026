package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Ports;

public class TurretConstants {

  public static final int kTurretID = Ports.kTurret;
  public static final boolean kTurretInverted = false;

  public static final double kTurretGearRatio = 82.0 * 5.0 / 10d;

  public static final double kTurretRadius = 0.20027; // TODO

  public static final double kP = 125.0; // 150
  public static final double kI = 0.0;
  public static final double kD = 0.0; // 0.0
  public static final double kG = 0.0; // 重力补偿
  public static final double kS = 0.1; // 静态前馈
  public static final double kV = 1.2; // 0.13 轨迹规划得到的目标速度8.0
  public static final double kA = 0.1; // 轨迹规划得到的目标加速度

  // ========== 角度限制 ==========
  public static final double kTurretMinAngle = Units.degreesToRadians(-270); // radius
  public static final double kTurretMaxAngle = Units.degreesToRadians(90.0); // radius
  public static final double kTurretInitialAngle = Units.degreesToRadians(-90.0);

  public static final double kTurretToleranceDeg = Units.degreesToRadians(3.5); // max is 4.6
  public static final double kTurretFixAngle = Units.degreesToRadians(90);

  // 炮塔的相对于机器中心的位置
  public static final Translation2d kTurretonRobotoffset = new Translation2d(-0.1225, -0.153);
  // 摄像头相对于炮塔中心的位置
  public static final Transform3d kCameraonTurretoffset =
      new Transform3d(
          -0.43,
          0.0, /// -0.43
          0.16237,
          new Rotation3d(
              Units.degreesToRadians(0), // roll
              Units.degreesToRadians(-24), // pitch
              Units.degreesToRadians(-180))); // yaw

  public static final Transform3d swerve2TurretOffset = new Transform3d(); // TODO

  public static final double kVelocityRadPerSec = Units.degreesToRadians(720.0);
  public static final double kAccelerationRadPerSecSq = Units.degreesToRadians(1200.0);
  public static final double kMaxWrapVelocity = Units.degreesToRadians(180.0);
  public static final double kMaxTurretTrackingVel = Units.radiansToDegrees(1400.0);
  public static final double kTurretWrapAngle = Units.radiansToDegrees(20.0);
  public static final double kMaxNormalVelocity = Units.radiansToDegrees(1000.0);

  // sim
  public static final double autoStartAngle = Units.degreesToRadians(0.0); // TODO
  public static final double jkMetersSquared = 10.06328;
  public static final double turretLength = 0.20054; // TODO noneed
  public static final double loopPeriodSecs = 0.02;

  public static final double turret_height_meters = 0.3004;
  public static final double kMaxAngularVelocityRadpersec = Units.degreesToRadians(720); // 2 rps
}
