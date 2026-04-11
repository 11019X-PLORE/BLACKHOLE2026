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
  // private final Supplier<Rotation2d> rotationSupplier;
  // private final DoubleArrayPublisher orientationPublisher;

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

  /**
   * Creates a new VisionIOLimelight.
   *
   * @param name The configured name of the Limelight.
   * @param rotationSupplier Supplier for the current estimated rotation, used for MegaTag 2.
   */
  public VisionIOLimelight(
      String name, double[] resulotion, PhysicalJoint baseJoint, Transform3d mountingOffset) {
    var table = NetworkTableInstance.getDefault().getTable(name);
    // this.rotationSupplier = rotationSupplier;
    // orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
    latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
    megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    tagPoseSubscriber =
        table.getDoubleArrayTopic("targetpose_cameraspace").subscribe(new double[] {});
    primaryIDSubscriber = table.getIntegerTopic("ty").subscribe(-1);

    rawdetectionsSubscriber = table.getDoubleArrayTopic("rawdetections").subscribe(new double[] {});

    num_pixels = resulotion[0] * resulotion[1];

    this.baseJoint = baseJoint;

    this.mountingOffset = mountingOffset;
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
    // orientationPublisher.accept(
    //     new double[] {rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});
    NetworkTableInstance.getDefault()
        .flush(); // Increases network traffic but recommended by Limelight

    // Read new pose observations from NetworkTables
    Set<Integer> tagIds = new HashSet<>();
    double[] rawDetections = rawdetectionsSubscriber.get();

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
              rawDetections.length > 0
                  ? VisionHelper.getMaxTagArea(rawDetections, 4) / num_pixels
                  : 0.0,

              // Observation type
              PoseObservationType.MEGATAG_1));
    }

    int primaryID = (int) primaryIDSubscriber.get();
    for (var rawSample : tagPoseSubscriber.readQueue()) {
      if (rawSample.value.length < 6) continue;

      // targetpose_cameraspace only has 6 values: x, y, z, rx, ry, rz
      // Use the megatag1 latency for timestamp, and primary ID for tag tracking
      tagIds.add(primaryID);

      // Compute distance from camera to tag using the translation components
      double tagDistance =
          Math.sqrt(
              rawSample.value[0] * rawSample.value[0]
                  + rawSample.value[1] * rawSample.value[1]
                  + rawSample.value[2] * rawSample.value[2]);

      poseObservations.add(
          new PoseObservation(
              // Timestamp, use NT server timestamp minus limelight latency
              rawSample.timestamp * 1.0e-6 - latencySubscriber.get() * 1.0e-3,

              // 3D pose estimate (camera-space tag pose)
              parsePose(rawSample.value),

              // Ambiguity (not available for single targetpose)
              0.0,

              // Tag count
              1,

              // Average tag distance
              tagDistance,
              rawDetections.length > 0
                  ? VisionHelper.getSingleTagArea(rawDetections, 4, primaryID, 0) / num_pixels
                  : 0.0,

              // Observation type
              PoseObservationType.CAMERA2TAG));
    }

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    // if (inputs.tagIds.length != 0) {
    //   inputs.tagIds[0] = primaryID;
    //   int i = 1;
    //   for (int id : tagIds) {
    //     if (id == primaryID) continue;
    //     System.out.println(primaryID);
    //     inputs.tagIds[i] = id;
    //     i++;
    //   }
    // }
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i] = id;
      i++;
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

  @Override
  public PhysicalJoint getBaseJoint() {
    return baseJoint;
  }

  @Override
  public Transform3d getMountingOffset() {
    return mountingOffset;
  }
}
