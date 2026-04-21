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
import edu.wpi.first.math.geometry.Pose3d;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.FullSubsystem;
import frc.robot.util.Geoffrey.PhysicalJoint;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.TrenchHelper;
import frc.robot.util.geometry.AllianceFlipUtil;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends FullSubsystem implements PhysicalJoint {
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

  // PathPlanner 配置
  private static final double ROBOT_MASS_KG = 75.0;
  private static final double ROBOT_MOI = 8;
  private static final double WHEEL_COF = 1.2;

  // Motor model constants for acceleration estimation
  private static final DCMotor DRIVE_MOTOR = DCMotor.getKrakenX60Foc(1);
  private static final double STALL_TORQUE_NM = DRIVE_MOTOR.stallTorqueNewtonMeters;
  private static final double FREE_SPEED_RAD_PER_SEC = DRIVE_MOTOR.freeSpeedRadPerSec;

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
  private final Module[] modules = new Module[4];
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);
  private final Field2d field = new Field2d();

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  private SwerveModulePosition[] lastModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private final PIDController headingPID = new PIDController(5.0, 0.0, 0.2);

  // 预分配对象
  private final SwerveModulePosition[] currentModulePositions = new SwerveModulePosition[4];
  private final SwerveModulePosition[] currentModuleDeltas = new SwerveModulePosition[4];
  private static final SwerveModuleState[] kEmptyModuleStates = new SwerveModuleState[] {};

  // 动力学数据
  private final PhysicalJoint.kinematics kinematicsData = new PhysicalJoint.kinematics();
  private PhysicalJoint base = PhysicalJoint.ground;

  private double rollOffset = 0.0;
  private double pitchOffset = 0.0;

  private TimeInterpolatableBuffer<Pose2d> robotPoseBuffer =
      TimeInterpolatableBuffer.createBuffer(2);

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
    super();

    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);
    PhoenixOdometryThread.getInstance().start();

    // PathPlanner 配置
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
        (activePath) ->
            Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0])));

    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose));

    headingPID.enableContinuousInput(-Math.PI, Math.PI);
    headingPID.setTolerance(Math.toRadians(1.5));

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
  public void updateInputsPeriodic() {
    odometryLock.lock();
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);

    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
      Logger.recordOutput("SwerveStates/Setpoints", kEmptyModuleStates);
      Logger.recordOutput("SwerveStates/SetpointsOptimized", kEmptyModuleStates);
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        currentModulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        currentModuleDeltas[moduleIndex] =
            new SwerveModulePosition(
                currentModulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                currentModulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = currentModulePositions[moduleIndex];
      }

      this.robotPoseBuffer.addSample(Timer.getFPGATimestamp(), getPose());
      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(currentModuleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, currentModulePositions);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
    field.setRobotPose(getPose());
    updateKinematics();
  }

  @Override
  public void periodic() {}

  @Override
  public void executePeriodic() {
    for (int i = 0; i < 4; i++) {
      Robot.batteryLogger.reportCurrentUsage(
          "Drive/Module" + i,
          true,
          modules[i].getDriveCurrentAmps(),
          modules[i].getSteerCurrentAmps());
    }
  }
  // --- 控制方法 ---
  public void runVelocity(ChassisSpeeds speeds) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  // --- SysId ---
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  // --- 路径规划 ---
  public Command pathfindToTargetPose(Pose2d targetPose) {
    PathConstraints constraints =
        new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(1080));
    return AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
  }

  // --- 状态获取 ---
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  private SwerveModuleState[] getModuleForces() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getForceState();
      states[i].speedMetersPerSecond *= modules.length;
    }
    return states;
  }

  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

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

  /**
   * Estimates the robot's field-relative acceleration using a motor torque model based on the
   * velocity setpoints and measured module speeds.
   *
   * <p>The motor torque model uses the DC motor torque-speed relationship:
   *
   * <ul>
   *   <li><b>Acceleration</b> (motor drives in direction of motion): available torque = τ_stall ×
   *       (1 − |ω| / ω_free). Starts at τ_stall at v=0, linearly to 0 at free speed.
   *   <li><b>Deceleration</b> (motor brakes against motion): available torque = τ_stall × (1 + |ω|
   *       / ω_free). Starts at τ_stall at v=0, linearly to 2×τ_stall at free speed.
   * </ul>
   *
   * Both are clamped by the stator current limit × kT. The velocity PID (kP=80 in TorqueCurrentFOC
   * mode) is aggressive enough that when a velocity error exists, the motor will command the
   * maximum available torque to close the gap. Per-module forces are summed to get robot-frame
   * acceleration (F/m, τ/I), then converted to field-relative.
   *
   * <p>This estimation is valid during normal driving with gradual acceleration profiles. It may be
   * inaccurate during sudden large steering changes.
   */
  public ChassisSpeeds getFieldAcceleration() {
    Translation2d[] modulePositions = kinematics.getModules();
    double totalFx = 0.0;
    double totalFy = 0.0;
    double totalTorque = 0.0;

    for (int i = 0; i < 4; i++) {
      Module module = modules[i];
      SwerveModuleState setpoint = module.getSetpointState();
      SwerveModuleState measured = module.getState();

      double setpointSpeed = setpoint.speedMetersPerSecond;
      double measuredSpeed = measured.speedMetersPerSecond;

      // Motor properties
      double kT = module.getDriveKt();
      double gearRatio = module.getDriveGearRatio();
      double wheelRadius = module.getWheelRadius();
      double statorCurrentLimit = module.getSlipCurrent();

      // Convert measured wheel speed to motor speed (rad/s)
      double motorSpeedRadPerSec = Math.abs(measuredSpeed) / wheelRadius * gearRatio;

      // Speed fraction [0, 1], clamped
      double speedFraction = Math.min(motorSpeedRadPerSec / FREE_SPEED_RAD_PER_SEC, 1.0);

      // Determine if motor is accelerating or decelerating
      // Accelerating: setpoint pushes further in direction of motion (or from rest)
      // Decelerating: setpoint opposes current motion
      boolean isDecelerating =
          (Math.abs(setpointSpeed) < Math.abs(measuredSpeed))
              || (setpointSpeed * measuredSpeed < 0);

      // Motor torque model (at the motor shaft, before gear reduction):
      // Accel:  τ = τ_stall × (1 - speedFraction)  [stall→0 as v goes 0→free]
      // Decel:  τ = τ_stall × (1 + speedFraction)  [stall→2×stall as v goes 0→free]
      double availableMotorTorque;
      if (isDecelerating) {
        availableMotorTorque = STALL_TORQUE_NM * (1.0 + speedFraction);
      } else {
        availableMotorTorque = STALL_TORQUE_NM * (1.0 - speedFraction);
      }

      // Clamp by stator current limit: max motor torque = currentLimit × kT
      double currentLimitTorque = statorCurrentLimit * kT;
      availableMotorTorque = Math.min(availableMotorTorque, currentLimitTorque);

      // Torque at wheel after gear reduction, then force at contact patch
      double wheelForce = availableMotorTorque * gearRatio / wheelRadius;

      // Velocity error determines force direction
      double velocityError = setpointSpeed - measuredSpeed;

      // The aggressive velocity PID will command max available torque when error exists.
      // For small errors, scale linearly to avoid discontinuity at the setpoint.
      double forceThreshold = 0.3; // m/s - below this error, scale force proportionally
      double forceMagnitude;
      if (Math.abs(velocityError) > forceThreshold) {
        forceMagnitude = wheelForce * Math.signum(velocityError);
      } else {
        forceMagnitude = wheelForce * (velocityError / forceThreshold);
      }

      // Force direction is along the module's setpoint steering angle
      double forceAngle = setpoint.angle.getRadians();
      double Fx = forceMagnitude * Math.cos(forceAngle);
      double Fy = forceMagnitude * Math.sin(forceAngle);

      totalFx += Fx;
      totalFy += Fy;
      // Torque about robot center: r × F
      totalTorque += -Fx * modulePositions[i].getY() + Fy * modulePositions[i].getX();
    }

    // Robot-relative acceleration: a = F/m, α = τ/I
    double ax = 0.5 * totalFx / ROBOT_MASS_KG;
    double ay = 0.5 * totalFy / ROBOT_MASS_KG;
    double alpha = 0.5 * totalTorque / ROBOT_MOI;

    Logger.recordOutput("Drive/EstimatedAccel", new double[] {ax, ay, alpha});

    // Convert to field-relative
    return ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(ax, ay, alpha), getRotation());
  }

  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

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

  public Rotation3d getRotation3d() {
    Rotation3d rotation =
        gyroInputs.rotation.rotateBy(TunerConstants.pigeonMountingOffset.getRotation());
    rotation =
        new Rotation3d(
            rotation.getX() + rollOffset, rotation.getY() + pitchOffset, rotation.getZ());
    rollOffset += (rotation.getX() > 0 ? -1 : 1) * TunerConstants.pigeonDriftRadsPerSec / 50.0;

    pitchOffset += (rotation.getY() > 0 ? -1 : 1) * TunerConstants.pigeonDriftRadsPerSec / 50.0;

    return new Rotation3d(rotation.getX(), rotation.getY(), getRotation().getRadians());
  }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  public void zeroHeading() {
    Translation2d currentTranslation = getPose().getTranslation();
    Rotation2d targetRotation = AllianceFlipUtil.apply(new Rotation2d());
    setPose(new Pose2d(currentTranslation, targetRotation));
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  public double getGyroRateDegPerSec() {
    return Math.toDegrees(gyroInputs.yawVelocityRadPerSec);
  }

  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  @AutoLogOutput(key = "Drive/InTrenchZone")
  public boolean isInTrenchZone() {
    return TrenchHelper.isInTrenchZone(() -> this.getPose());
  }

  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }

  // --- PhysicalJoint 接口实现 ---

  @Override
  public void updateKinematics() {
    Pose2d currentPose = getPose();
    kinematicsData.forwardKinematic =
        new Transform3d(
            new Translation3d(
                currentPose.getTranslation().getX(), currentPose.getTranslation().getY(), 0),
            getRotation3d());

    Logger.recordOutput("Drive/forwardKinematic", kinematicsData.forwardKinematic);
    Logger.recordOutput("Drive/3dPose", Pose3d.kZero.transformBy(kinematicsData.forwardKinematic));

    ChassisSpeeds currentSpeeds = getChassisSpeeds();
    SimpleMatrix vel = kinematicsData.localVelocity;
    vel.set(0, currentSpeeds.vxMetersPerSecond);
    vel.set(1, currentSpeeds.vyMetersPerSecond);
    vel.set(2, 0);
    vel.set(3, 0);
    vel.set(4, 0);
    vel.set(5, currentSpeeds.omegaRadiansPerSecond);

    ChassisSpeeds currentAcceleration = getFieldAcceleration();
    Logger.recordOutput("Drive/Acc", currentAcceleration);
    SimpleMatrix acc = kinematicsData.localAcceleration;
    acc.set(0, currentAcceleration.vxMetersPerSecond);
    acc.set(1, currentAcceleration.vyMetersPerSecond);
    acc.set(2, 0);
    acc.set(3, 0);
    acc.set(4, 0);
    acc.set(5, currentAcceleration.omegaRadiansPerSecond);
  }

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
