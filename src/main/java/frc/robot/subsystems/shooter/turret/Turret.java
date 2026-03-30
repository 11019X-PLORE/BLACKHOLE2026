package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.turret.TurretIO.TurretIOOutputMode;
import frc.robot.subsystems.shooter.turret.TurretIO.TurretIOOutputs;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PhysicalJoint;
import frc.robot.util.geometry.AllianceFlipUtil;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase implements PhysicalJoint{
  private static final double trackOverlapMargin = Units.degreesToRadians(10);
  private static final double trackMinAngle = TurretConstants.kTurretMinAngle - trackOverlapMargin;
  private static final double trackMaxAngle = TurretConstants.kTurretMaxAngle + trackOverlapMargin;

  private static final LoggedTunableNumber testVelocity =
      new LoggedTunableNumber("turret/TestVelocity");
  public static final LoggedTunableNumber kP = new LoggedTunableNumber("turret/PID/kP");
  public static final LoggedTunableNumber kI = new LoggedTunableNumber("turret/PID/kI");
  public static final LoggedTunableNumber kD = new LoggedTunableNumber("turret/PID/kD");
  public static final LoggedTunableNumber kA = new LoggedTunableNumber("turret/PID/kA");
  public static final LoggedTunableNumber kV = new LoggedTunableNumber("turret/PID/kV");
  public static final LoggedTunableNumber kS = new LoggedTunableNumber("turret/PID/kS");
  public static final LoggedTunableNumber kG = new LoggedTunableNumber("turret/PID/kG");
  private static final LoggedTunableNumber kFixAngle = new LoggedTunableNumber("turret/kFixAngle");
  public static final LoggedTunableNumber toleranceDeg =
      new LoggedTunableNumber("turret/ToleranceDeg");

  static {
    if (Robot.isSimulation()) {
      kP.initDefault(35);
      kI.initDefault(0.0);
      kD.initDefault(0.1);
      kS.initDefault(0.0);
      kV.initDefault(0.0);
      kA.initDefault(0.0);
      kG.initDefault(0.0);
      toleranceDeg.initDefault(1.0);
      kFixAngle.initDefault(90.0);
      testVelocity.initDefault(TurretConstants.kMaxAngularVelocityRadpersec);
    } else {
      kP.initDefault(TurretConstants.kP);
      kI.initDefault(TurretConstants.kI);
      kD.initDefault(TurretConstants.kD);
      kS.initDefault(TurretConstants.kS);
      kV.initDefault(TurretConstants.kV);
      kA.initDefault(TurretConstants.kA);
      kG.initDefault(TurretConstants.kG);
      toleranceDeg.initDefault(TurretConstants.kTurretToleranceDeg);
      kFixAngle.initDefault(TurretConstants.kTurretFixAngle);
      testVelocity.initDefault(TurretConstants.kVelocityRadPerSec);
    }
  }

  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final TurretIOOutputs outputs = new TurretIOOutputs();

  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> chassisSpeedSupplier;

  public enum TurretGoal {
    IDLE,
    TRACKING,
    FIXED_ANGLE,
    PASSING,
    ZEROING,
    TEST
  }

  @Getter @Setter @AutoLogOutput private TurretGoal goal = TurretGoal.IDLE;

  private double lastGoalAngle = 0.0;
  private double currentSetpoint = 0.0; // 平滑后的目标位置
  private double wrapTargetAngle = 0.0; // 记录回环的最终目的地
  private boolean isWrapping = false; // 是否处于回环锁定状态

  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Alert disconnected =
      new Alert("Turret motor disconnected!", Alert.AlertType.kWarning);

  @Getter @AutoLogOutput private boolean turretZeroed = false;

  @Getter
  @Accessors(fluent = true)
  @AutoLogOutput
  private boolean atGoal = false;

  private final String name;
  private final Translation2d robotToTurret;
  private Pose2d turretPose = new Pose2d();
  private final PhysicalJoint.kinematics kinematicsData = new PhysicalJoint.kinematics();
  private PhysicalJoint base = null;

  public Turret(
      TurretIO io,
      Translation2d robotToTurret,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> chassisSpeedSupplier,
      String name) {
    this.name = name;
    this.io = io;
    this.robotToTurret = robotToTurret;
    this.poseSupplier = poseSupplier;
    this.chassisSpeedSupplier = chassisSpeedSupplier;
    io.setPID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get(), kG.get());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("turret/turret" + name, inputs);
    disconnected.set(!motorConnectedDebouncer.calculate(inputs.turretMotorConnected));
    updateTunables();
    calculateTurretPose();
    updateKinematics();

    if (DriverStation.isDisabled() || !turretZeroed) {
      outputs.mode = TurretIOOutputMode.COAST;
      outputs.velocity = 0.0;
      atGoal = false;
      // 禁用时强制同步位置，防止下次启用时猛跳
      lastGoalAngle = inputs.positionRads;
      currentSetpoint = inputs.positionRads;
      isWrapping = false;
    } else {
      switch (goal) {
        case IDLE -> {
          outputs.mode = TurretIOOutputMode.COAST;
          outputs.velocity = 0.0;
          atGoal = true;
          currentSetpoint = inputs.positionRads;
        }
        case TRACKING -> {
          var params = ShotCalculator.getInstance().getParameters();
          runTrackingLogic(params.turretAngle(), params.turretVelocity());
        }
        case FIXED_ANGLE -> {
          var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
          Rotation2d targetFieldAngle =
              (alliance == Alliance.Red)
                  ? Rotation2d.fromDegrees(180.0)
                  : Rotation2d.fromDegrees(0.0);
          double compensateVel = -chassisSpeedSupplier.get().omegaRadiansPerSecond;
          runTrackingLogic(targetFieldAngle, compensateVel);
        }
        case PASSING -> {
          double compensateVel = -chassisSpeedSupplier.get().omegaRadiansPerSecond;
          runTrackingLogic(getBestPassingAngle(), compensateVel);
        }
        case ZEROING -> {
          Rotation2d robotRot = poseSupplier.get().getRotation();
          Rotation2d targetFieldAngle = robotRot.plus(Rotation2d.fromDegrees(-90.0));
          runTrackingLogic(targetFieldAngle, 0.0);
        }
        case TEST -> {
          double targetRelativeRads =
              MathUtil.clamp(
                  kFixAngle.get(),
                  TurretConstants.kTurretMinAngle,
                  TurretConstants.kTurretMaxAngle);
          outputs.mode = TurretIOOutputMode.CLOSED_LOOP;
          outputs.position = targetRelativeRads; // 直接给电机相对位置
          outputs.velocity = testVelocity.get();
        }
      }
    }

    io.applyOutputs(outputs);
    Logger.recordOutput("turret/turret" + name + "/atGoal", atGoal);
    Logger.recordOutput("turret/turret" + name + "/IsWrapping", isWrapping);
    Logger.recordOutput("turret/turret" + name + "/CurrentSetpoint", currentSetpoint);
  }

  private void runTrackingLogic(Rotation2d goalAngleFieldRelative, double goalVelocity) {
    // 1. 获取当前底盘朝向
    Rotation2d robotAngle = poseSupplier.get().getRotation();
    // 计算目标相对于车身的“原始”角度
    double targetRads = goalAngleFieldRelative.minus(robotAngle).getRadians();

    // 2. 寻找最近的合法角度 (保持搜索逻辑)
    // 这步确保炮塔在 [-270, 90] 的物理墙内找到离当前位置最近的等效点
    boolean hasBestAngle = false;
    double bestAngle = 0;
    for (int i = -2; i < 3; i++) {
      double potentialSetpoint = targetRads + (i * 2.0 * Math.PI);
      if (potentialSetpoint >= TurretConstants.kTurretMinAngle
          && potentialSetpoint <= TurretConstants.kTurretMaxAngle) {
        if (!hasBestAngle
            || Math.abs(lastGoalAngle - potentialSetpoint) < Math.abs(lastGoalAngle - bestAngle)) {
          bestAngle = potentialSetpoint;
          hasBestAngle = true;
        }
      }
    }

    // 强制 Clamp 到物理边界
    if (!hasBestAngle) {
      bestAngle =
          MathUtil.clamp(
              targetRads, TurretConstants.kTurretMinAngle, TurretConstants.kTurretMaxAngle);
    }

    // 更新上一次的目标点，确保搜索逻辑的连续性
    lastGoalAngle = bestAngle;

    outputs.mode = TurretIOOutputMode.CLOSED_LOOP;
    outputs.position = bestAngle;
    outputs.velocity = goalVelocity;

    double goalStateAngle =
        MathUtil.clamp(bestAngle, TurretConstants.kTurretMinAngle, TurretConstants.kTurretMaxAngle);

    atGoal = EqualsUtil.epsilonEquals(bestAngle, inputs.positionRads, toleranceDeg.get());

    Logger.recordOutput("turret/turret" + name + "/GoalPositionRad", bestAngle);
    Logger.recordOutput("turret/turret" + name + "/SetpointPositionRad", goalStateAngle);
  }

  private void updateTunables() {
    if (kP.hasChanged(hashCode())
        || kD.hasChanged(hashCode())
        || kA.hasChanged(hashCode())
        || kV.hasChanged(hashCode())) {
      io.setPID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get(), kG.get());
    }
  }

  public void zero() {
    turretZeroed = true;
    lastGoalAngle = inputs.positionRads;
    currentSetpoint = inputs.positionRads;
  }

  public Command setGoalCommand(TurretGoal newGoal) {
    return Commands.runOnce(() -> this.goal = newGoal, this);
  }

  public Command zeroCommand() {
    return runOnce(
            () -> {
              turretZeroed = true;
              lastGoalAngle = inputs.positionRads;
              currentSetpoint = inputs.positionRads;
              this.goal = TurretGoal.IDLE;
            })
        .ignoringDisable(true);
  }

  public void calculateTurretPose() {
    if (turretZeroed) {
      turretPose =
          poseSupplier
              .get()
              .transformBy(
                  new Transform2d(robotToTurret, Rotation2d.fromRadians(inputs.positionRads)));
    }
  }

  public Pose2d getTurretpose() {
    return turretPose;
  }

  public double getPosition() {
    return inputs.positionRads;
  }

  public double getVelocity() {
    return inputs.velocityRadsPerSec;
  }

  public Translation2d getRobotToTurret() {
    return robotToTurret;
  }

  private Rotation2d getBestPassingAngle() {
    // 1. 定义两个基础目标点（基于蓝方原点：X=0是蓝墙，Y增大是向左）
    Translation2d blueLeftTarget = new Translation2d(1.874, 5.49);
    Translation2d blueRightTarget = new Translation2d(1.874, 2.17);

    // 2. 根据当前联盟自动翻转坐标
    Translation2d leftTarget = AllianceFlipUtil.apply(blueLeftTarget);
    Translation2d rightTarget = AllianceFlipUtil.apply(blueRightTarget);

    // 3. 获取当前机器人平面的位置
    Translation2d robotTrans = poseSupplier.get().getTranslation();

    // 4. 选择距离最近的那个点
    Translation2d bestTarget =
        (robotTrans.getDistance(leftTarget) < robotTrans.getDistance(rightTarget))
            ? leftTarget
            : rightTarget;

    // 5. 记录选中的目标点到日志，方便在 AdvantageScope 中通过 Pose2d 观察
    Logger.recordOutput(
        "turret/turret" + name + "/PassingTargetUsed", new Pose2d(bestTarget, new Rotation2d()));

    // 6. 返回从机器人指向该目标点的角度
    return bestTarget.minus(robotTrans).getAngle();
  }

  public Pose3d getCameraPoseRobotSpace() {
    Pose3d turretBase =
        new Pose3d(robotToTurret.getX(), robotToTurret.getY(), 0.3004, new Rotation3d());
    Rotation3d turretRotation = new Rotation3d(0.0, 0.0, inputs.positionRads);
    Transform3d cameraOnTurret = TurretConstants.kCameraonTurretoffset;
    return turretBase
        .transformBy(new Transform3d(new Translation3d(), turretRotation))
        .transformBy(cameraOnTurret);
  }

  public Rotation2d getFieldRotation() {
    return poseSupplier.get().getRotation().plus(new Rotation2d(getPosition()));
  }

  public void launchFuel() {
    io.launchFuel();
  }

  //TODO call this in robot container after creating turret and drive, to set up the kinematics chain
  public void setBase(PhysicalJoint base) {
    this.base = base;
  }

  @Override
  public void updateKinematics(){
    kinematicsData.forwardKinematic = TurretConstants.swerve2TurretOffset.plus(
      new Transform3d(new Translation3d(), new Rotation3d(0, 0, getPosition())));

    kinematicsData.localVelocity = new SimpleMatrix(6, 1);
    kinematicsData.localAcceleration = new SimpleMatrix(6, 1);
  };

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
