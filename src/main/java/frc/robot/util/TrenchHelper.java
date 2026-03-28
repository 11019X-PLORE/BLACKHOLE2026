package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.FieldConstants;
import frc.robot.util.geometry.AllianceFlipUtil;
import java.util.Comparator;
import java.util.List;
import java.util.function.Supplier;

public class TrenchHelper {

  public record TrenchZone(Translation2d center, double sizeX, double sizeY) {
    public boolean contains(Pose2d pose) {
      return Math.abs(pose.getX() - center.getX()) <= sizeX / 2.0
          && Math.abs(pose.getY() - center.getY()) <= sizeY / 2.0;
    }
  }

  // Trench 尺寸 (来自常量)
  private static final double trenchDepthX = FieldConstants.LeftTrench.depth;
  private static final double trenchWidthY = FieldConstants.LeftTrench.openingWidth;

  public static final List<TrenchZone> ALL_TRENCHES =
      List.of(
          // 1. 蓝方左侧 Trench (靠近场地上方)
          new TrenchZone(
              new Translation2d(
                  FieldConstants.LinesVertical.hubCenter,
                  FieldConstants.fieldWidth - trenchWidthY / 2.0),
              trenchDepthX,
              trenchWidthY),

          // 2. 蓝方右侧 Trench (靠近场地下方)
          new TrenchZone(
              new Translation2d(FieldConstants.LinesVertical.hubCenter, trenchWidthY / 2.0),
              trenchDepthX,
              trenchWidthY),

          // 3. 红方左侧 Trench (从蓝方看是在右半场上方)
          new TrenchZone(
              new Translation2d(
                  FieldConstants.LinesVertical.oppHubCenter,
                  FieldConstants.fieldWidth - trenchWidthY / 2.0),
              trenchDepthX,
              trenchWidthY),

          // 4. 红方右侧 Trench (从蓝方看是在右半场下方)
          new TrenchZone(
              new Translation2d(FieldConstants.LinesVertical.oppHubCenter, trenchWidthY / 2.0),
              trenchDepthX,
              trenchWidthY));

  /**
   * 判断当前机器人是否在任意一个 Trench 区域内 (包含敌方半场)
   *
   * @param currentRobotPoseSupplier 机器人位姿的提供者 (例如 drive::getPose)
   * @return 如果在 Trench 区域内返回 true
   */
  public static boolean isInTrenchZone(Supplier<Pose2d> currentRobotPoseSupplier) {
    Pose2d currentPose = currentRobotPoseSupplier.get();

    Pose2d absolutePose =
        AllianceFlipUtil.shouldFlip() ? AllianceFlipUtil.apply(currentPose) : currentPose;

    for (TrenchZone trench : ALL_TRENCHES) {
      if (trench.contains(absolutePose)) return true;
    }
    return false;
  }

  /**
   * 根据机器人当前位置，动态计算穿过最近 Trench 的目标点。
   *
   * @param currentRobotPoseSupplier 机器人位姿的提供者 (例如 drive::getPose)
   * @return 最终在真实场地坐标系下的目标 Pose2d
   */
  public static Pose2d getTrenchTargetPose(Supplier<Pose2d> currentRobotPoseSupplier) {
    Pose2d currentPose = currentRobotPoseSupplier.get();

    // 1. 确保在蓝方绝对坐标系下进行计算
    Pose2d absolutePose =
        AllianceFlipUtil.shouldFlip() ? AllianceFlipUtil.apply(currentPose) : currentPose;

    // 2. 找到距离机器人最近的 Trench (全场 4 个中选 1 个)
    TrenchZone closestTrench =
        ALL_TRENCHES.stream()
            .min(
                Comparator.comparingDouble(
                    trench -> absolutePose.getTranslation().getDistance(trench.center())))
            .orElse(ALL_TRENCHES.get(0));

    // 3. 判断在 Trench 的左侧还是右侧，决定穿梭方向
    double targetX;
    if (absolutePose.getX() < closestTrench.center().getX()) {
      // 机器人在左侧(前方)，目标点在中心点右侧(后方) 1米处
      targetX = closestTrench.center().getX() + (trenchDepthX / 2.0) + 1.0;
    } else {
      // 机器人在右侧(后方)，目标点在中心点左侧(前方) 1米处
      targetX = closestTrench.center().getX() - (trenchDepthX / 2.0) - 1.0;
    }

    // 4. 生成绝对坐标系下的目标位姿 (不用管车头的方向)
    Rotation2d targetHeading = absolutePose.getRotation();

    Pose2d absoluteTargetPose = new Pose2d(targetX, closestTrench.center().getY(), targetHeading);

    // 5. 将计算出的绝对目标点，翻转回当前联盟的相对坐标系（喂给 PathPlanner）
    return AllianceFlipUtil.shouldFlip()
        ? AllianceFlipUtil.apply(absoluteTargetPose)
        : absoluteTargetPose;
  }
}
