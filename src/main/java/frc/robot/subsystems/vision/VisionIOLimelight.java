package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.util.Geoffrey.PhysicalJoint;
import frc.robot.util.Geoffrey.VisionHelper;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

/** IO implementation for real Limelight hardware. */
public class VisionIOLimelight implements VisionIO {
  private final DoubleSubscriber latencySubscriber;
  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;
  private final IntegerSubscriber primaryIDSubscriber;
  private final DoubleArraySubscriber megatag1Subscriber;
  private final DoubleArraySubscriber tagPoseSubscriber;
  private final DoubleArraySubscriber rawdetectionsSubscriber;

  private final double num_pixels;
  private final PhysicalJoint baseJoint;
  private final Transform3d mountingOffset;

  public VisionIOLimelight(
      String name, double[] resulotion, PhysicalJoint baseJoint, Transform3d mountingOffset) {
    var table = NetworkTableInstance.getDefault().getTable(name);

    latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
    megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    tagPoseSubscriber =
        table.getDoubleArrayTopic("targetpose_cameraspace").subscribe(new double[] {});

    // 确保订阅的是 "tid" 而不是 "ty"，获取主目标的 ID
    primaryIDSubscriber = table.getIntegerTopic("ty").subscribe(-1);

    rawdetectionsSubscriber = table.getDoubleArrayTopic("rawdetections").subscribe(new double[] {});

    num_pixels = resulotion[0] * resulotion[1];
    this.baseJoint = baseJoint;
    this.mountingOffset = mountingOffset;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // 1. 更新连接状态检查
    inputs.connected =
        ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

    // 2. 基础目标角度观察
    inputs.latestTargetObservation =
        new TargetObservation(
            Rotation2d.fromDegrees(txSubscriber.get()), Rotation2d.fromDegrees(tySubscriber.get()));

    // 刷新网络表
    NetworkTableInstance.getDefault().flush();

    Set<Integer> tagIds = new HashSet<>();
    double[] rawDetections = rawdetectionsSubscriber.get();
    List<PoseObservation> poseObservations = new LinkedList<>();

    // 获取当前底盘/关节的真实 Yaw (弧度)
    // 这里利用 baseJoint (你传入的 PhysicalJoint) 来获取不带视觉误差的航向角
    double robotYawRads = baseJoint.getGlobalPose().getRotation().getZ();

    // --- 处理 MegaTag 1 ---
    for (var rawSample : megatag1Subscriber.readQueue()) {
      if (rawSample.value.length == 0) continue;

      // 提取所有看到的 Tag ID
      for (int i = 11; i < rawSample.value.length; i += 7) {
        tagIds.add((int) rawSample.value[i]);
      }

      // 获取视觉原始 X, Y
      double x = rawSample.value[0];
      double y = rawSample.value[1];

      // ⭐ 核心优化：强制融合陀螺仪 Yaw，丢弃视觉算出的 Roll/Pitch (防止后空翻)
      Pose3d correctedPose = new Pose3d(x, y, 0.0, new Rotation3d(0.0, 0.0, robotYawRads));

      // 单标签歧义过滤
      if (rawSample.value[7] == 1) { // 如果只看到一个 Tag
        // 如果视觉算出的高度 Z 超过 30cm 或者 Roll/Pitch 倾斜过大，说明解算反转了，直接丢弃
        if (Math.abs(rawSample.value[2]) > 0.3 || Math.abs(rawSample.value[3]) > 15.0) {
          continue;
        }
      }

      poseObservations.add(
          new PoseObservation(
              rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,
              correctedPose,
              rawSample.value.length >= 18 ? rawSample.value[17] : 0.0,
              (int) rawSample.value[7],
              rawSample.value[9],
              rawDetections.length > 0
                  ? VisionHelper.getMaxTagArea(rawDetections, 4) / num_pixels
                  : 0.0,
              PoseObservationType.MEGATAG_1));
    }

    // 获取当前主 ID
    int primaryID = (int) primaryIDSubscriber.get();

    // --- 处理 Camera2Tag (单目标相机坐标系) ---
    for (var rawSample : tagPoseSubscriber.readQueue()) {
      if (rawSample.value.length == 0) continue;
      // 单目标模式下通常只处理 primaryID
      if (primaryID != -1) tagIds.add(primaryID);

      poseObservations.add(
          new PoseObservation(
              rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,
              parsePose(rawSample.value),
              0.0,
              1,
              rawSample.value.length > 2 ? rawSample.value[2] : 0.0,
              rawDetections.length > 0
                  ? VisionHelper.getSingleTagArea(rawDetections, 4, primaryID, 0) / num_pixels
                  : 0.0,
              PoseObservationType.CAMERA2TAG));
    }

    // 3. 保存观测值到 inputs
    inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);

    // ⭐ 4. 修复崩溃：安全地处理 tagIds 数组
    if (tagIds.isEmpty()) {
      inputs.tagIds = new int[0];
    } else {
      inputs.tagIds = new int[tagIds.size()];
      int writeIndex = 0;

      // 如果 primaryID 有效且在集合中，将其放在首位
      if (primaryID != -1 && tagIds.contains(primaryID)) {
        inputs.tagIds[writeIndex++] = primaryID;
      }

      // 填充剩余 ID
      for (int id : tagIds) {
        if (id == primaryID) continue;
        inputs.tagIds[writeIndex++] = id;
      }
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

  @Override
  public PhysicalJoint getBaseJoint() {
    return baseJoint;
  }

  @Override
  public Transform3d getMountingOffset() {
    return mountingOffset;
  }
}
