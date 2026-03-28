package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class VisionIOLimelightTurret implements VisionIO {
  private final DoubleArrayPublisher orientationPublisher;
  private final DoubleSubscriber latencySubscriber;
  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;
  private final DoubleArraySubscriber megatag1Subscriber;
  private final DoubleArrayPublisher cameraPosePublisher;

  // 用于坐标变换和速度限制
  private final Supplier<Pose3d> cameraToRobotSupplier;
  private final DoubleSupplier turretVelocitySupplier; // 新增：炮塔速度
  private final double maxTurretVelocityDegPerSec; // 新增：最大允许速度

  /**
   * @param name Limelight 名称
   * @param cameraToRobotSupplier 返回相机相对于机器人的位置 (包含 Turret 旋转)
   * @param turretVelocitySupplier 返回 Turret 当前的角速度 (单位: 度/秒 或 弧度/秒，需统一)
   * @param maxTurretVelocityDegPerSec 允许进行视觉更新的最大炮塔速度 (单位: 度/秒)
   */
  public VisionIOLimelightTurret(
      String name,
      Supplier<Pose3d> cameraToRobotSupplier,
      DoubleSupplier turretVelocitySupplier,
      double maxTurretVelocityDegPerSec) {

    var table = NetworkTableInstance.getDefault().getTable(name);
    this.cameraToRobotSupplier = cameraToRobotSupplier;
    this.turretVelocitySupplier = turretVelocitySupplier;
    this.maxTurretVelocityDegPerSec = maxTurretVelocityDegPerSec;

    cameraPosePublisher = table.getDoubleArrayTopic("camerapose_robotspace_set").publish();
    orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
    latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
    megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});

    // 初始化：向 Limelight 发送全 0 的偏移。
    // 这意味着 Limelight 返回的 botpose_wpiblue 实际上是 **相机镜头在场地的绝对位置**。
    // 我们将在 Java 代码中处理 Turret 的旋转，而不是依赖 Limelight 的内部计算。
    cameraPosePublisher.accept(new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // 1. 连接状态检查
    inputs.connected =
        ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

    // 2. 基础目标数据
    inputs.latestTargetObservation =
        new TargetObservation(
            Rotation2d.fromDegrees(txSubscriber.get()), Rotation2d.fromDegrees(tySubscriber.get()));

    // 3. 速度限制检查
    if (Math.abs(turretVelocitySupplier.getAsDouble()) > maxTurretVelocityDegPerSec) {
      inputs.poseObservations = new PoseObservation[0];
      inputs.tagIds = new int[0];
      return;
    }

    // 4. 强制归零 (防止 Limelight 内部叠加偏移)
    cameraPosePublisher.accept(new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    // 5. 准备坐标变换数据
    Set<Integer> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new ArrayList<>();

    // 获取 RIO 计算的 Camera -> Robot 变换 (包含 Turret 旋转)
    Pose3d robotToCameraPose3d = cameraToRobotSupplier.get();

    // 1. 提取 2D 变换 (RobotCenter -> CameraLens on 2D plane)
    Transform2d robotToCamera2d =
        new Transform2d(
            robotToCameraPose3d.getTranslation().toTranslation2d(),
            robotToCameraPose3d.getRotation().toRotation2d());

    for (var rawSample : megatag1Subscriber.readQueue()) {
      if (rawSample.value.length == 0) continue;

      // Tag ID 提取
      for (int i = 11; i < rawSample.value.length; i += 7) {
        tagIds.add((int) rawSample.value[i]);
      }

      // 2. 解析 Limelight 返回的相机绝对坐标 (Field Space)
      Pose3d cameraFieldPose3d = parsePose(rawSample.value);

      // 3. 将相机坐标也降维到 2D
      Pose2d cameraFieldPose2d = cameraFieldPose3d.toPose2d();

      // 4. 执行 2D 逆变换：从相机位置推回机器人中心
      // Robot = Camera * (Robot->Camera)^-1
      Pose2d robotFieldPose2d = cameraFieldPose2d.transformBy(robotToCamera2d.inverse());

      // 5. 重组回 Pose3d (强制 Z=0, Roll=0, Pitch=0)
      // 这样 AdvantageScope 里机器人永远是平的，不会乱飞
      Pose3d robotFieldPoseCorrected =
          new Pose3d(
              robotFieldPose2d.getX(),
              robotFieldPose2d.getY(),
              0.0, // 强制高度为 0
              new Rotation3d(0.0, 0.0, robotFieldPose2d.getRotation().getRadians()));

      // 调试日志
      // Logger.recordOutput("Vision/Summary/CameraFieldPose2d", cameraFieldPose2d);
      // Logger.recordOutput(
      //     "Vision/Summary/RobotFieldPoseCorrected", robotFieldPoseCorrected);

      poseObservations.add(
          new PoseObservation(
              rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,
              robotFieldPoseCorrected, // 使用修正后的 Pose
              rawSample.value.length >= 18 ? rawSample.value[17] : 0.0,
              (int) rawSample.value[7],
              rawSample.value[9],
              PoseObservationType.MEGATAG_1));
    }

    // 写入 inputs
    inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);

    // 提取 Tag IDs
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }

  private static Pose3d parsePose(double[] rawLLArray) {
    return new Pose3d(
        rawLLArray[0],
        rawLLArray[1],
        rawLLArray[2],
        new Rotation3d(
            Units.degreesToRadians(rawLLArray[3]),
            Units.degreesToRadians(rawLLArray[4]),
            Units.degreesToRadians(rawLLArray[5])));
  }
}
