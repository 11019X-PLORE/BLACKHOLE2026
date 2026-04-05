package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "limelight-fix";
  public static String camera1Name = "limelight-turret";

  // Robot to camera transforms(sim)
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCamera0 =
      new Transform3d(0.0, 0.0, 0.616, new Rotation3d(0.0, -0.4, 0.0));
  public static Transform3d robotToCamera1 =
      new Transform3d(-0.0, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));

  // camera to Robot transforms(Real)
  public static Pose3d cameraToRobot1 =
      new Pose3d( // 常量，与 WebUI 一致
          -0.04371,
          -0.39998,
          0.3661,
          new Rotation3d(
              Units.degreesToRadians(0), Units.degreesToRadians(30), Units.degreesToRadians(90)));
  public static double visionCutoffSpeed = Units.degreesToRadians(45);
  // TODO:
  //   public static final Transform3d turretToCamera =
  //       new Transform3d(new Translation3d(0.5, 0.6, 0.0), new Rotation3d(0.0, 10, 0.0));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevFactor = 0.5; // More stable than full 3D solve
  public static double angularStdDevFactor = 1e9; // Don't trust heading

  public static double latencyStdDev = 0.02; // Seconds, for vision timestamp uncertainty
  /**
   * Limelight 目标面积（ta）到视觉测量偏差系数的插值查找表。
   *
   * <p>物理含义说明：
   *
   * <ul>
   *   <li>ta 表示 AprilTag 在图像中的相对面积
   *   <li>ta 越大，目标越近，视觉位姿解算越可靠
   *   <li>ta 越小，目标越远，视觉测量噪声越大
   * </ul>
   *
   * <p>该查找表用于根据目标距离动态调整视觉测量噪声， 使位姿估计器在近距离时更信任视觉，在远距离时降低视觉权重。
   */
  public static InterpolatingDoubleTreeMap taToDev = new InterpolatingDoubleTreeMap();

  /**
   * 运动模型（里程计）的状态标准差（过程噪声）。
   *
   * <p>顺序为：(x 位置 [米]，y 位置 [米]，航向角 [弧度])
   *
   * <p>这些参数描述了系统对“轮子里程计 + 陀螺仪预测结果”的信任程度：
   *
   * <ul>
   *   <li>数值越小：越信任底盘运动模型
   *   <li>数值越大：越允许视觉对位姿进行修正
   * </ul>
   *
   * <p>调参经验：
   *
   * <ul>
   *   <li>若地面抓地力好、打滑少，可适当减小 x / y
   *   <li>若陀螺仪稳定、漂移小，可适当减小 heading
   *   <li>若位姿难以被视觉拉回，说明该值可能偏小
   * </ul>
   */
  public static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.5, 0.5, 0.1); // 0.1 0.1 0.1

  /**
   * 视觉位姿测量的基础标准差。
   *
   * <p>顺序为：(x 位置 [米]，y 位置 [米]，航向角 [弧度])
   *
   * <p>该参数表示在未考虑目标距离之前， 单次视觉位姿测量的理论不确定度。
   *
   * <p>数值通常设置得较为保守，以避免由于：
   *
   * <ul>
   *   <li>远距离 AprilTag
   *   <li>视角极端
   *   <li>标签遮挡或误识别
   *   <li>角度不确定度被设置成无限大 完全相信陀螺仪
   * </ul>
   *
   * 导致的位姿跳变。
   *
   * <p>在实际使用中，通常会结合 {@link #taToDev} 根据 ta 对该标准差进行动态缩放。
   */
  public static final Matrix<N3, N1> visionStdDevs =
      VecBuilder.fill(0.9, 0.9, Double.POSITIVE_INFINITY);
}
