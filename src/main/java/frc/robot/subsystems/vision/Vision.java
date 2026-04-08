package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.FullSubsystem;

public class Vision extends FullSubsystem {
  private final Drive drive;
  // private final Drive drive;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  private final TimeInterpolatableBuffer<Pose3d>[] robot2cameraPoseBuffer;

  public Vision(Drive drive, VisionIO... io) {
    // this.drive = drive;
    this.drive = drive;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }

    robot2cameraPoseBuffer =
        new TimeInterpolatableBuffer[io.length]; // Separate buffer for each camera
    for (int i = 0; i < io.length; i++) {
      robot2cameraPoseBuffer[i] = TimeInterpolatableBuffer.createBuffer(2.0);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {}

  @Override
  public void periodicAfterScheduler() {
    return;

    // for (int i = 0; i < io.length; i++) {
    //   io[i].updateInputs(inputs[i]);
    //   Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);

    //   // Compute the camera pose in the robot (base) frame:
    //   //   T_robot_to_camera = T_world_to_robot^-1 ⊕ T_world_to_camera
    //   // where T_world_to_camera = mountingEnd.globalTip ⊕ mountingOffset
    //   Transform3d cameraGlobal =
    //       io[i].getBaseJoint().getGlobalPose().plus(io[i].getMountingOffset());
    //   Transform3d robotGlobal = drive.getGlobalPose();
    //   // robot^-1 ⊕ camera  →  camera expressed in robot frame
    //   Pose3d cameraInRobot = new Pose3d().transformBy(robotGlobal.inverse().plus(cameraGlobal));
    //   this.robot2cameraPoseBuffer[i].addSample(Timer.getFPGATimestamp(), cameraInRobot);
    // }

    // // Initialize logging values
    // List<Pose3d> allTagPoses = new LinkedList<>();
    // List<Pose3d> allRobotPoses = new LinkedList<>();
    // List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    // List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // // Loop over cameras
    // for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
    //   // Update disconnected alert
    //   disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

    //   // Initialize logging values
    //   List<Pose3d> tagPoses = new LinkedList<>();
    //   List<Pose3d> robotPoses = new LinkedList<>();
    //   List<Pose3d> robotPosesAccepted = new LinkedList<>();
    //   List<Pose3d> robotPosesRejected = new LinkedList<>();

    //   // Add tag poses
    //   for (int tagId : inputs[cameraIndex].tagIds) {
    //     var tagPose = aprilTagLayout.getTagPose(tagId);
    //     if (tagPose.isPresent()) {
    //       tagPoses.add(tagPose.get());
    //     }
    //   }

    //   SimpleMatrix baseV = io[cameraIndex].getBaseJoint().getGlobalVelocity();

    //   double baseSpeed = Math.hypot(baseV.get(0, 0), baseV.get(1, 0));
    //   double baseAngularSpeed = Math.abs(baseV.get(5, 0));

    //   // Loop over pose observations
    //   for (var observation : inputs[cameraIndex].poseObservations) {

    //     Pose3d robot2cameraPose =
    //         this.robot2cameraPoseBuffer[cameraIndex].getSample(observation.timestamp()).get();

    //     if (robot2cameraPose == null) {
    //       continue; // Skip if we don't have a valid robot-to-camera transform at the observation
    //       // timestamp
    //     }

    //     var sampleRobotPose = drive.getPose(observation.timestamp());

    //     Pose3d visionPose3d = null;
    //     if (observation.type() == VisionIO.PoseObservationType.MEGATAG_1) {
    //       visionPose3d =
    //           observation
    //               .pose()
    //               .transformBy(new Transform3d(new Pose3d(), robot2cameraPose).inverse());
    //     } else if (observation.type() == VisionIO.PoseObservationType.CAMERA2TAG) {
    //       // Pin-Point estimation
    //       Pose3d robotToTagPose =
    //           robot2cameraPose.plus(new Transform3d(new Pose3d(), observation.pose()));
    //       Translation2d fieldToTagTranslation =
    //           AprilTagLayoutType.OFFICIAL
    //               .getLayout()
    //               .getTagPose(inputs[cameraIndex].tagIds[0])
    //               .get()
    //               .getTranslation()
    //               .toTranslation2d();
    //       Translation2d robotToTagTranslation =
    // robotToTagPose.getTranslation().toTranslation2d();

    //       Translation2d fieldToRobot =
    //           fieldToTagTranslation.minus(
    //               robotToTagTranslation.rotateBy(sampleRobotPose.getRotation()));
    //       // TODO update std dev calculation to account for distance and tag count, and add a
    //       // separate angular std dev
    //       // double translationDev = (odometryStateStdDevs.get(2, 0)+robotToTag.stdDevs.get(2,
    // 0)) *
    //       // robotToTagTranslation.getNorm() + robotToTag.stdDevs.get(0, 0);
    //       // var stdDevs = new Matrix<>(VecBuilder.fill(translationDev, translationDev, 1e+12));
    //       visionPose3d =
    //           new Pose3d(
    //               new Translation3d(fieldToRobot.getX(), fieldToRobot.getY(), 0),
    //               new Rotation3d(0, 0, sampleRobotPose.getRotation().getRadians()));

    //     } else {
    //       continue; // Skip unsupported observation types
    //     }

    //     // Check whether to reject pose
    //     boolean rejectPose =
    //         observation.tagCount() == 0 // Must have at least one tag
    //             || (observation.tagCount() == 1
    //                 && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
    //             || Math.abs(visionPose3d.getZ()) > maxZError // Must have realistic Z coordinate
    //             || Math.abs(
    //                     visionPose3d.getRotation().getZ()
    //                         - sampleRobotPose.getRotation().getRadians())
    //                 > maxYawDifference // Must not have large yaw difference from odometry
    //             // Must be within the field boundaries
    //             || visionPose3d.getX() < 0.0
    //             || visionPose3d.getX() > aprilTagLayout.getFieldLength()
    //             || visionPose3d.getY() < 0.0
    //             || visionPose3d.getY() > aprilTagLayout.getFieldWidth();

    //     // Add pose to log
    //     robotPoses.add(visionPose3d);
    //     if (rejectPose) {
    //       robotPosesRejected.add(visionPose3d);
    //     } else {
    //       robotPosesAccepted.add(visionPose3d);
    //     }

    //     // Skip if rejected
    //     if (rejectPose) {
    //       continue;
    //     }

    //     // 1. 基于距离和 Tag 数量计算基础因子 (距离越远，平方级增加标准差)
    //     double stdDevFactor = 1 / (observation.maxArea() + 1e-6); // 避免除以零，面积越大（目标越近），因子越小

    //     double linearStdDev =
    //         linearStdDevBaseline
    //             + stdDevFactor * linearStdDevFactor
    //             + baseSpeed * VisionConstants.latencyStdDev;
    //     double angularStdDev =
    //         angularStdDevBaseline
    //             + stdDevFactor * angularStdDevFactor
    //             + baseAngularSpeed * VisionConstants.latencyStdDev;

    //     // 3. 应用每个摄像头的独立调整系数
    //     if (cameraIndex < cameraStdDevFactors.length) {
    //       linearStdDev *= cameraStdDevFactors[cameraIndex];
    //       angularStdDev *= cameraStdDevFactors[cameraIndex];
    //     }

    //     // Send vision observation
    //     drive.addVisionMeasurement(
    //         visionPose3d.toPose2d(),
    //         observation.timestamp(),
    //         VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
    //   }

    //   // Log camera metadata
    //   Logger.recordOutput(
    //       "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
    //       tagPoses.toArray(new Pose3d[0]));
    //   Logger.recordOutput(
    //       "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
    //       robotPoses.toArray(new Pose3d[0]));
    //   Logger.recordOutput(
    //       "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
    //       robotPosesAccepted.toArray(new Pose3d[0]));
    //   Logger.recordOutput(
    //       "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
    //       robotPosesRejected.toArray(new Pose3d[0]));
    //   allTagPoses.addAll(tagPoses);
    //   allRobotPoses.addAll(robotPoses);
    //   allRobotPosesAccepted.addAll(robotPosesAccepted);
    //   allRobotPosesRejected.addAll(robotPosesRejected);
    // }

    // // Log summary data
    // Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
    // Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
    // Logger.recordOutput(
    //     "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
    // Logger.recordOutput(
    //     "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
  }
}
