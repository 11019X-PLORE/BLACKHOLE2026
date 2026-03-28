package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.geometry.AllianceFlipUtil;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.function.Supplier;

/** 爬升目标选择器 用于在比赛末期，根据机器人当前的位置，自动选择最近的爬升点（Reef/Chain 等）。 */
public class ClimbTargetSelector {

  // 定义三个爬升点的静态常量 (X, Y, Rotation2d(弧度))
  public static final Pose2d CLIMB_POINT_TOP =
      new Pose2d(1.53, 2.92, new Rotation2d(Units.degreesToRadians(-90)));

  public static final Pose2d CLIMB_POINT_BOTTOM =
      new Pose2d(1.53, 3.9, new Rotation2d(Units.degreesToRadians(-90)));

  // 将它们装入一个列表中，方便遍历比较
  private static final List<Pose2d> CLIMB_POSES =
      Arrays.asList(CLIMB_POINT_TOP, CLIMB_POINT_BOTTOM);

  /**
   * 传入一个底盘位姿的 Supplier，计算并返回距离机器人当前最近的爬升点。
   *
   * @param currentRobotPoseSupplier 机器人位姿的提供者 (例如 drive::getPose)
   * @return 最近的爬升点 Pose2d
   */
  public static Pose2d getNearestClimbPose(Supplier<Pose2d> currentRobotPoseSupplier) {

    Pose2d currentPose = currentRobotPoseSupplier.get();
    Pose2d poseForComparison =
        AllianceFlipUtil.shouldFlip() ? AllianceFlipUtil.apply(currentPose) : currentPose;

    // 在蓝方半场算出最近的点
    Pose2d nearestBluePose =
        CLIMB_POSES.stream()
            .min(
                Comparator.comparingDouble(
                    targetPose ->
                        poseForComparison
                            .getTranslation()
                            .getDistance(targetPose.getTranslation())))
            .orElse(CLIMB_POINT_BOTTOM);

    // 关键：算出是哪个点后，如果是红方，就把这个点翻转到红方半场交出去！
    return AllianceFlipUtil.shouldFlip()
        ? AllianceFlipUtil.apply(nearestBluePose)
        : nearestBluePose;
  }
}
