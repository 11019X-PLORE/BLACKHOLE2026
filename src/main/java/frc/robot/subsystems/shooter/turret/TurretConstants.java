package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Ports;
import frc.robot.util.Geoffrey.PhysicalJoint;

public class TurretConstants {

  public static final int kTurretID = Ports.kTurret;
  public static final boolean kTurretInverted = true;

  public static final double kTurretGearRatio = 41.111111111111;

  public static final double kTurretRadius = 0.20027; // TODO

  public static final double kP = 3000.0; // 150
  public static final double kI = 0.0;
  public static final double kD = 80.0; // 0.0
  public static final double kG = 0.0; // 重力补偿
  public static final double kS = 0.1; // 静态前馈
  public static final double kV = 120; // 0.13 轨迹规划得到的目标速度8.0
  public static final double kA = 0.0; // 轨迹规划得到的目标加速度

  public static final double kInertiaTurret = 0.1; // TODO
  public static final double kT = 0.01537;

  // ========== 角度限制 ==========
  public static final double kTurretMinAngle = Units.degreesToRadians(-270); // radius
  public static final double kTurretMaxAngle = Units.degreesToRadians(90.0); // radius
  public static final double kTurretInitialAngle = Units.degreesToRadians(0);

  public static final double kTurretToleranceDeg = Units.degreesToRadians(10); // max is 4.6
  public static final double kTurretFixAngle = Units.degreesToRadians(-90.0);

  // 炮塔的相对于机器中心的位置
  public static final Translation2d kTurretonRobotoffset = new Translation2d(-0.158, 0);
  // 摄像头相对于炮塔中心的位置
  public static final Transform3d kCameraonTurretoffset =
      new Transform3d(
          new Translation3d(0.17946, 0, 0.06767),
          new Rotation3d(Math.PI, Math.toRadians(-26.08), 0));
  public static final Transform3d swerve2TurretOffset =
      new Transform3d(new Translation3d(-0.158, 0, 0.407), new Rotation3d(0, 0, 0.0));

  public static final double kVelocityRadPerSec = Units.degreesToRadians(845.0);
  public static final double kAccelerationRadPerSecSq = Units.degreesToRadians(1200.0);
  public static final double kMaxTurretTrackingVel = Units.radiansToDegrees(845.0);

  // sim
  public static final double autoStartAngle = Units.degreesToRadians(0.0); // TODO
  public static final double jkMetersSquared = 10.06328;
  public static final double turretLength = 0.20054; // TODO noneed
  public static final double loopPeriodSecs = 0.02;

  public static final double turret_height_meters = 0.3004;
  public static final double kMaxAngularVelocityRadpersec = Units.degreesToRadians(720); // 2 rps

  public static PhysicalJoint swerve2TurretStructure =
      PhysicalJoint.getStructureJoint(swerve2TurretOffset);
}
