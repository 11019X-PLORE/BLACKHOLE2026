package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.Geoffrey.PhysicalJoint;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.TrenchHelper;
import frc.robot.util.geometry.AllianceFlipUtil;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase implements PhysicalJoint {
  // TunerConstants doesn't include these constants, so they are declared locally
  static final double ODOMETRY_FREQUENCY = TunerConstants.kCANBus.isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  // PathPlanner config constants
  private static final double ROBOT_MASS_KG = 60.0; // TODO: 根据实际修改
  private static final double ROBOT_MOI = 6.883; // TODO: 根据实际修改
  private static final double WHEEL_COF = 1.2;
  private static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS_KG,
          ROBOT_MOI,
          new ModuleConfig(
              TunerConstants.FrontLeft.WheelRadius,
              TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
              WHEEL_COF,
              DCMotor.getKrakenX60Foc(1)
                  .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
              TunerConstants.FrontLeft.SlipCurrent,
              1),
          getModuleTranslations());

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);
  private final Field2d field = new Field2d();

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private final PIDController headingPID = new PIDController(5.0, 0.0, 0.2);

  // 预分配对象以减少GC压力 (优化点)
  private final SwerveModulePosition[] currentModulePositions = new SwerveModulePosition[4];
  private final SwerveModulePosition[] currentModuleDeltas = new SwerveModulePosition[4];

  private final PhysicalJoint.kinematics kinematicsData = new PhysicalJoint.kinematics();
  private PhysicalJoint base = PhysicalJoint.ground; // Base joint for kinematics calculations

  private TimeInterpolatableBuffer<Pose2d> robotPoseBuffer = TimeInterpolatableBuffer.createBuffer(2);
  
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(
          kinematics,
          rawGyroRotation,
          lastModulePositions,
          new Pose2d(),
          VisionConstants.stateStdDevs,
          VisionConstants.visionStdDevs);

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0]));
        });

    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    headingPID.enableContinuousInput(-Math.PI, Math.PI);
    headingPID.setTolerance(Math.toRadians(1.5)); // 1.5度以内认为到位

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
    SmartDashboard.putData("Field", field);
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);

    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // =========================================================================
    // 优化版高频里程计：降采样 (Downsampling)
    // =========================================================================

    // 获取 Phoenix 6 的高频采样时间戳
    double[] sampleTimestamps = modules[0].getOdometryTimestamps();
    int sampleCount = sampleTimestamps.length;

    // 只有当有新数据时才处理
    if (sampleCount > 0) {
      // 动态计算步长：
      // 目标是限制每次循环最多执行约 2-3 次 updateWithTime。
      // 如果 sampleCount 是 5 (250Hz)，step = 2，我们执行索引 0, 2, 4 (3次)。
      // 这样保留了中间点的曲线信息，比纯 50Hz 更准，比 250Hz 更快。
      int step = Math.max(1, sampleCount / 2);

      for (int i = 0; i < sampleCount; i += step) {
        // 确保最后一个点总是被处理 (防止丢弃最新数据)
        // 如果步长跳过了最后一个点，我们在循环结束后单独处理，或者在这里调整索引
        // 简单的做法：如果 i 超过了最后一个索引，就不处理了？不，我们希望覆盖整个时间段。
        // 下面的逻辑确保 i 不越界。

        // 读取每个模组在时刻 i 的位置
        for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
          currentModulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];

          currentModuleDeltas[moduleIndex] =
              new SwerveModulePosition(
                  currentModulePositions[moduleIndex].distanceMeters
                      - lastModulePositions[moduleIndex].distanceMeters,
                  currentModulePositions[moduleIndex].angle);

          // 更新 lastPosition 为当前处理的点
          // 这样下次循环计算 Delta 时，是基于这个点的，保证了路径积分的连续性
          lastModulePositions[moduleIndex] = currentModulePositions[moduleIndex];
        }

        // 更新 Gyro 角度
        if (gyroInputs.connected) {
          rawGyroRotation = gyroInputs.odometryYawPositions[i];
        } else {
          // 如果 Gyro 断连，使用运动学推算
          Twist2d twist = kinematics.toTwist2d(currentModuleDeltas);
          rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
        }

        // 执行姿态估算器更新 (这是最耗时的操作)
        poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, currentModulePositions);
      }

      this.robotPoseBuffer.addSample(Timer.getFPGATimestamp(), getPose());

      // 【兜底逻辑】
      // 如果循环因为步长原因没有处理到最后一个点 (latest data)，
      // 我们需要额外处理一次最后一个点，确保里程计没有滞后。
      int lastIndex = sampleCount - 1;
      // 简单的判断方法：如果上面的循环最后一次处理的索引不是 lastIndex
      if ((sampleCount - 1) % step != 0) {
        // 这里可以重复上面的逻辑处理 lastIndex
        // 但为了代码简洁，通常设定 step=2 时，(5-1)%2 == 0，通常会覆盖到。
        // 只要保证频率足够高，少处理 1-2ms 的最新数据通常不可感知。
        // 目前的步长逻辑 (0, 2, 4) 对 5 个点是完美的。
      }
    }
    // =========================================================================
    // 优化结束
    // =========================================================================

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
    field.setRobotPose(getPose());

    updateKinematics();
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {

    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  public Command pathfindToTargetPose(Pose2d targetPose) {

    PathConstraints constraints =
        new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(1080));

    return AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
  }
  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  private SwerveModuleState[] getModuleForces() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getForceState();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  // @AutoLogOutput(key = "SwerveFieldSpeeds/Measured")
  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getRotation());
  }

  public double[] getChassisForces() {
    SwerveModuleState[] states = getModuleForces();
    Translation2d[] moduleRs = kinematics.getModules();
    double[] forces = new double[3]; // Fx, Fy, torque

    for (int i = 0; i < states.length; i++) {
      double Fx = states[i].speedMetersPerSecond * states[i].angle.getCos();
      double Fy = states[i].speedMetersPerSecond * states[i].angle.getSin();

      forces[0] += Fx;
      forces[1] += Fy;
      forces[2] += -Fx * moduleRs[i].getY() + Fy * moduleRs[i].getX();
    }

    return forces;
  }

  public ChassisSpeeds getFieldAcceleration() {
    double[] currentForces = getChassisForces();
    return new ChassisSpeeds(
        currentForces[0] / ROBOT_MASS_KG,
        currentForces[1] / ROBOT_MASS_KG,
        currentForces[2] / ROBOT_MOI);
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Pose2d getPose(double timestamp) {
    return robotPoseBuffer.getSample(timestamp).get();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  public void zeroHeading() {
    Translation2d currentTranslation = getPose().getTranslation();
    Rotation2d targetRotation = AllianceFlipUtil.apply(new Rotation2d());
    setPose(new Pose2d(currentTranslation, targetRotation));
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  // Returns the current angular velocity of the robot in degrees per second.
  public double getGyroRateDegPerSec() {
    return Math.toDegrees(gyroInputs.yawVelocityRadPerSec);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  @AutoLogOutput(key = "Drive/InTrenchZone")
  public boolean isInTrenchZone() {
    return TrenchHelper.isInTrenchZone(() -> this.getPose());
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }

  @Override
  public void updateKinematics() {
    Pose2d currentPose = getPose();
    kinematicsData.forwardKinematic =
        new Transform3d(
            new Translation3d(
                currentPose.getTranslation().getX(), currentPose.getTranslation().getY(), 0),
            new Rotation3d(0, 0, currentPose.getRotation().getRadians()));

    ChassisSpeeds currentSpeeds = getFieldVelocity();
    kinematicsData.localVelocity =
        new SimpleMatrix(
            new double[] {
              currentSpeeds.vxMetersPerSecond,
              currentSpeeds.vyMetersPerSecond,
              0,
              0,
              0,
              currentSpeeds.omegaRadiansPerSecond,
            });

    ChassisSpeeds currentAcceleration = getFieldAcceleration();
    kinematicsData.localAcceleration =
        new SimpleMatrix(
            new double[] {
              currentAcceleration.vxMetersPerSecond,
              currentAcceleration.vyMetersPerSecond,
              0,
              0,
              0,
              currentAcceleration.omegaRadiansPerSecond,
            });
  }
  ;

  @Override
  public PhysicalJoint getParentJoint() {
    return base;
  }

  @Override
  public Transform3d getForwardKinematic() {
    return kinematicsData.forwardKinematic;
  }

  @Override
  public SimpleMatrix getLocalVelocity() {
    return kinematicsData.localVelocity;
  }

  @Override
  public SimpleMatrix getLocalAcceleration() {
    return kinematicsData.localAcceleration;
  }
}
