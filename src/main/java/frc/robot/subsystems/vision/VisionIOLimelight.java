package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

/** IO implementation for real Limelight hardware. */
public class VisionIOLimelight implements VisionIO {
  private final Supplier<Rotation2d> rotationSupplier;
  private final DoubleArrayPublisher orientationPublisher;

  private final DoubleSubscriber latencySubscriber;
  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;
  private final DoubleArraySubscriber megatag1Subscriber;
  private final DoubleArraySubscriber megatag2Subscriber;

  /**
   * Creates a new VisionIOLimelight.
   *
   * @param name The configured name of the Limelight.
   * @param rotationSupplier Supplier for the current estimated rotation, used for MegaTag 2.
   */
  public VisionIOLimelight(String name, Supplier<Rotation2d> rotationSupplier) {
    var table = NetworkTableInstance.getDefault().getTable(name);
    this.rotationSupplier = rotationSupplier;
    orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
    latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
    megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    megatag2Subscriber =
        table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Update connection status based on whether an update has been seen in the last
    // 250ms
    inputs.connected =
        ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

    // Update target observation
    inputs.latestTargetObservation =
        new TargetObservation(
            Rotation2d.fromDegrees(txSubscriber.get()), Rotation2d.fromDegrees(tySubscriber.get()));

    // Update orientation for MegaTag 2
    orientationPublisher.accept(
        new double[] {rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});
    NetworkTableInstance.getDefault()
        .flush(); // Increases network traffic but recommended by Limelight

    // Read new pose observations from NetworkTables
    Set<Integer> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    for (var rawSample : megatag1Subscriber.readQueue()) {
      if (rawSample.value.length == 0) continue;
      for (int i = 11; i < rawSample.value.length; i += 7) {
        tagIds.add((int) rawSample.value[i]);
      }
      poseObservations.add(
          new PoseObservation(
              // Timestamp, based on server timestamp of publish and latency
              rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,

              // 3D pose estimate
              parsePose(rawSample.value),

              // Ambiguity, using only the first tag because ambiguity isn't applicable for
              // multitag
              rawSample.value.length >= 18 ? rawSample.value[17] : 0.0,

              // Tag count
              (int) rawSample.value[7],

              // Average tag distance
              rawSample.value[9],

              // Observation type
              PoseObservationType.MEGATAG_1));
    }
    for (var rawSample : megatag2Subscriber.readQueue()) {
      if (rawSample.value.length == 0) continue;
      for (int i = 11; i < rawSample.value.length; i += 7) {
        tagIds.add((int) rawSample.value[i]);
      }
      poseObservations.add(
          new PoseObservation(
              // Timestamp, based on server timestamp of publish and latency
              rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,

              // 3D pose estimate
              parsePose(rawSample.value),

              // Ambiguity, zeroed because the pose is already disambiguated
              0.0,

              // Tag count
              (int) rawSample.value[7],

              // Average tag distance
              rawSample.value[9],

              // Observation type
              PoseObservationType.MEGATAG_2));
    }

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }

  /** Parses the 3D pose from a Limelight botpose array. */
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

// package frc.robot.subsystems.vision;

// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.networktables.DoubleArrayPublisher;
// import edu.wpi.first.networktables.DoubleArraySubscriber;
// import edu.wpi.first.networktables.DoubleSubscriber;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.RobotController;
// import java.util.HashSet;
// import java.util.LinkedList;
// import java.util.List;
// import java.util.Set;
// import java.util.function.Supplier;

// /** IO implementation for real Limelight hardware. */
// public class VisionIOLimelight implements VisionIO {
//   private final Supplier<Rotation2d> rotationSupplier;
//   private final DoubleArrayPublisher orientationPublisher;

//   private final DoubleSubscriber latencySubscriber;
//   private final DoubleSubscriber txSubscriber;
//   private final DoubleSubscriber tySubscriber;
//   private final DoubleArraySubscriber megatag1Subscriber;
//   private final DoubleArraySubscriber megatag2Subscriber;

//   /**
//    * Creates a new VisionIOLimelight.
//    *
//    * @param name The configured name of the Limelight.
//    * @param rotationSupplier Supplier for the current estimated rotation, used for MegaTag 2.
//    */
//   public VisionIOLimelight(String name, Supplier<Rotation2d> rotationSupplier) {
//     var table = NetworkTableInstance.getDefault().getTable(name);
//     this.rotationSupplier = rotationSupplier;
//     orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
//     latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
//     txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
//     tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
//     megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
//     megatag2Subscriber =
//         table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
//   }

//   @Override
//   public void updateInputs(VisionIOInputs inputs) {
//     // Update connection status based on whether an update has been seen in the last
//     // 250ms
//     inputs.connected =
//         ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

//     // Update target observation
//     inputs.latestTargetObservation =
//         new TargetObservation(
//             Rotation2d.fromDegrees(txSubscriber.get()),
// Rotation2d.fromDegrees(tySubscriber.get()));

//     // Update orientation for MegaTag 2
//     orientationPublisher.accept(
//         new double[] {rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});
//     NetworkTableInstance.getDefault()
//         .flush(); // Increases network traffic but recommended by Limelight

//     // Read new pose observations from NetworkTables
//     Set<Integer> tagIds = new HashSet<>();
//     List<PoseObservation> poseObservations = new LinkedList<>();
//     for (var rawSample : megatag1Subscriber.readQueue()) {
//       if (rawSample.value.length == 0) continue;
//       for (int i = 11; i < rawSample.value.length; i += 7) {
//         tagIds.add((int) rawSample.value[i]);
//       }

//       // 1. 获取 Limelight 原始位置 (X, Y)
//       double llX = rawSample.value[0];
//       double llY = rawSample.value[1];
//       // 强制 Z, Roll, Pitch 为 0 (保证底盘在地板上平移)
//       Pose3d robotFieldPose =
//           new Pose3d(llX, llY, 0.0, new Rotation3d(0.0, 0.0,
// rotationSupplier.get().getRadians()));
//       int tagCount = (int) rawSample.value[7];
//       if (tagCount == 1) {
//         // 如果只有 1 个标签，但解算出来的 Z 轴或倾斜度异常，直接丢弃
//         // 这能有效防止“幽灵车”乱跳
//         if (Math.abs(rawSample.value[2]) > 0.3
//             || Math.abs(rawSample.value[4]) > 15.0) { // 简单检查高度和俯仰
//           continue;
//         }
//       }
//       poseObservations.add(
//           new PoseObservation(
//               rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,
//               robotFieldPose,
//               rawSample.value.length >= 18 ? rawSample.value[17] : 0.0,
//               tagCount,
//               rawSample.value[9],
//               PoseObservationType.MEGATAG_1));
//     }
//     for (var rawSample : megatag2Subscriber.readQueue()) {
//       if (rawSample.value.length == 0) continue;
//       for (int i = 11; i < rawSample.value.length; i += 7) {
//         tagIds.add((int) rawSample.value[i]);
//       }
//       poseObservations.add(
//           new PoseObservation(
//               // Timestamp, based on server timestamp of publish and latency
//               rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,

//               // 3D pose estimate
//               parsePose(rawSample.value),

//               // Ambiguity, zeroed because the pose is already disambiguated
//               0.0,

//               // Tag count
//               (int) rawSample.value[7],

//               // Average tag distance
//               rawSample.value[9],

//               // Observation type
//               PoseObservationType.MEGATAG_2));
//     }

//     // Save pose observations to inputs object
//     inputs.poseObservations = new PoseObservation[poseObservations.size()];
//     for (int i = 0; i < poseObservations.size(); i++) {
//       inputs.poseObservations[i] = poseObservations.get(i);
//     }

//     // Save tag IDs to inputs objects
//     inputs.tagIds = new int[tagIds.size()];
//     int i = 0;
//     for (int id : tagIds) {
//       inputs.tagIds[i++] = id;
//     }
//   }

//   /** Parses the 3D pose from a Limelight botpose array. */
//   private static Pose3d parsePose(double[] rawLLArray) {
//     return new Pose3d(
//         rawLLArray[0],
//         rawLLArray[1],
//         rawLLArray[2],
//         new Rotation3d(
//             Units.degreesToRadians(rawLLArray[3]),
//             Units.degreesToRadians(rawLLArray[4]),
//             Units.degreesToRadians(rawLLArray[5])));
//   }
// }
