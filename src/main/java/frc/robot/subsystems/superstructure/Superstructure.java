package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.subsystems.extension.Extension;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerGoal;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.Flywheel.FlywheelGoal;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.Hood.HoodGoal;
import frc.robot.subsystems.shooter.hood.HoodConstants;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.Turret.TurretGoal;
import frc.robot.subsystems.shooter.turret.TurretConstants;
import frc.robot.subsystems.triggers.Triggers;
import frc.robot.subsystems.triggers.Triggers.TriggersGoal;
import frc.robot.util.FullSubsystem;
import frc.robot.util.Geoffrey.PhysicalJoint;
import frc.robot.util.Geoffrey.ShooterSetpoint;
import frc.robot.util.Geoffrey.TrajectoryCalculator;
import frc.robot.util.Geoffrey.TrajectoryConfig;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.geometry.AllianceFlipUtil;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends FullSubsystem {

  private final Turret turret;
  private final Hood hood;
  private final Flywheel flywheel;
  private final Intake intake;
  private final Extension extension;
  private final Triggers triggers;
  private final Indexer indexer;
  private final LED led;

  public enum SuperstructureState {
    IDLE,
    INTAKE,
    SPIT,
    SHOOTSPIT,
    SPITSTOP,
    ACTIVESHOOTING,
    SHOOTING,
    SHOOTING_FIXED,
    PASSING,
    TRENCH,
    TEST,
  }

  @Getter @AutoLogOutput private SuperstructureState currentState = SuperstructureState.IDLE;
  @AutoLogOutput private SuperstructureState fallbackState = SuperstructureState.IDLE;

  private final Map<SuperstructureState, Supplier<Command>> stateMap;
  private final BooleanSupplier inTrenchZoneSupplier;

  private static final LoggedTunableNumber kMeasureShootV =
      new LoggedTunableNumber("Flywheel/kMeasureVelocity");

  public Superstructure(
      Turret turret,
      Hood hood,
      Flywheel flywheel,
      Intake intake,
      Extension extension,
      Triggers triggers,
      Indexer indexer,
      LED led,
      BooleanSupplier inTrenchZoneSupplier) {
    this.turret = turret;
    this.hood = hood;
    this.flywheel = flywheel;
    this.intake = intake;
    this.extension = extension;
    this.triggers = triggers;
    this.indexer = indexer;
    this.led = led;
    this.inTrenchZoneSupplier = inTrenchZoneSupplier;

    kMeasureShootV.initDefault(0);

    stateMap =
        Map.ofEntries(
            Map.entry(
                SuperstructureState.IDLE,
                () ->
                    Commands.parallel(
                        turret.setGoalCommand(TurretGoal.IDLE),
                        hood.setGoalCommand(HoodGoal.IDLE),
                        flywheel.setGoalCommand(FlywheelGoal.IDLE),
                        triggers.setGoalCommand(TriggersGoal.STOP),
                        indexer.setGoalCommand(IndexerGoal.STOP))),
            Map.entry(
                SuperstructureState.SPIT,
                () ->
                    Commands.parallel(
                        Commands.runOnce(
                            () -> {
                              triggers.setGoal(TriggersGoal.OUTTAKE);
                              indexer.setGoal(IndexerGoal.OUTTAKE);
                            },
                            triggers,
                            indexer))),
            Map.entry(
                SuperstructureState.INTAKE,
                () ->
                    Commands.parallel(
                        Commands.runOnce(
                            () -> {
                              triggers.setGoal(TriggersGoal.STOP);
                              indexer.setGoal(IndexerGoal.INTAKE);
                            },
                            triggers,
                            indexer))),
            Map.entry(
                SuperstructureState.SPITSTOP,
                () ->
                    Commands.parallel(
                        Commands.runOnce(
                            () -> {
                              triggers.setGoal(TriggersGoal.STOP);
                              indexer.setGoal(IndexerGoal.STOP);
                            },
                            triggers,
                            indexer))),
            Map.entry(
                SuperstructureState.SHOOTSPIT,
                () ->
                    Commands.parallel(
                        flywheel.setGoalCommand(FlywheelGoal.FIXED_VELOCITY),
                        Commands.runOnce(
                            () -> {
                              triggers.setGoal(TriggersGoal.SHOOT);
                              indexer.setGoal(IndexerGoal.SHOOT);
                            },
                            triggers,
                            indexer))),
            Map.entry(
                SuperstructureState.ACTIVESHOOTING,
                () ->
                    Commands.parallel(
                        turret.setGoalCommand(TurretGoal.TRACKING),
                        hood.setGoalCommand(HoodGoal.ZEROING),
                        flywheel.setGoalCommand(FlywheelGoal.ACTIVE),
                        Commands.run(
                            () -> {
                              Translation2d targetPos =
                                  AllianceFlipUtil.apply(
                                      FieldConstants.Hub.topCenterPoint.toTranslation2d());
                              ShooterSetpoint sp =
                                  ShooterSetpoint.makeSetpoint(
                                      TurretConstants.swerve2TurretStructure,
                                      targetPos,
                                      FieldConstants.hMax,
                                      HoodConstants.kHoodMinAngle,
                                      HoodConstants.kHoodMaxAngle,
                                      TrajectoryConfig.getHubConfig());

                              turret.runPositionFOCLogic(
                                  sp.turretPositionRadians,
                                  sp.turretVelocityRadsPerSec,
                                  sp.turretAccelerationRadsPerSecSquared,
                                  0.0);

                              triggers.setGoal(TriggersGoal.STOP);
                              indexer.setGoal(IndexerGoal.STOP);

                              Logger.recordOutput(
                                  "shooterSetpoint/turretPositionRadians",
                                  sp.turretPositionRadians);
                              Logger.recordOutput(
                                  "shooterSetpoint/turretVelocityRadsPerSec",
                                  sp.turretVelocityRadsPerSec);
                              Logger.recordOutput(
                                  "shooterSetpoint/turretAccelerationRadsPerSecSquared",
                                  sp.turretAccelerationRadsPerSecSquared);
                            },
                            triggers,
                            indexer))),
            Map.entry(
                SuperstructureState.SHOOTING,
                () ->
                    Commands.parallel(
                        turret.setGoalCommand(TurretGoal.TRACKING),
                        hood.setGoalCommand(HoodGoal.TRACKING),
                        flywheel.setGoalCommand(FlywheelGoal.TRACKING),
                        Commands.run(
                            () -> {
                              Translation2d targetPos =
                                  AllianceFlipUtil.apply(
                                      FieldConstants.Hub.topCenterPoint.toTranslation2d());
                              ShooterSetpoint sp =
                                  ShooterSetpoint.makeSetpoint(
                                      TurretConstants.swerve2TurretStructure,
                                      targetPos,
                                      FieldConstants.hMax,
                                      HoodConstants.kHoodMinAngle,
                                      HoodConstants.kHoodMaxAngle,
                                      TrajectoryConfig.getHubConfig());

                              double lookAheadTime = 0.02;
                              turret.runPositionFOCLogic(
                                  sp.turretPositionRadians
                                      + (lookAheadTime * sp.turretVelocityRadsPerSec),
                                  sp.turretVelocityRadsPerSec
                                      + (lookAheadTime * sp.turretAccelerationRadsPerSecSquared),
                                  sp.turretAccelerationRadsPerSecSquared,
                                  // 0,
                                  // 0,
                                  0.0);
                              hood.runPositionFOCLogic(
                                  sp.hoodPositionRadians
                                      + (lookAheadTime * sp.hoodVelocityRadsPerSec),
                                  sp.hoodVelocityRadsPerSec
                                      + (lookAheadTime * sp.hoodAccelerationRadsPerSecSquared),
                                  sp.hoodAccelerationRadsPerSecSquared,
                                  0.0);

                              double radPerSec =
                                  TrajectoryCalculator.getFlywheelSetpoint(
                                          sp.shooterVelocityMetersPerSec
                                              + (lookAheadTime
                                                  * sp.shooterAccelerationMetersPerSecSquared))
                                      // kMeasureShootV.get()
                                      / FlywheelConstants.kFlywheelRadius;
                              double radPerSec2 =
                                  TrajectoryCalculator.getFlywheelAcceleration(
                                          sp.shooterVelocityMetersPerSec,
                                          sp.shooterAccelerationMetersPerSecSquared)
                                      / FlywheelConstants.kFlywheelRadius;
                              flywheel.runVelocityFOCLogic(radPerSec, radPerSec2, 0.0);

                              if (isReadyToShoot()) {
                                triggers.setGoal(TriggersGoal.SHOOT);
                                indexer.setGoal(IndexerGoal.SHOOT);
                              } else {
                                triggers.setGoal(TriggersGoal.STOP);
                                indexer.setGoal(IndexerGoal.STOP);
                              }

                              Logger.recordOutput(
                                  "shooterSetpoint/shooterVelocityMetersPerSec",
                                  sp.shooterVelocityMetersPerSec);
                              Logger.recordOutput(
                                  "shooterSetpoint/turretPositionRadians",
                                  sp.turretPositionRadians);
                              Logger.recordOutput(
                                  "shooterSetpoint/turretVelocityRadsPerSec",
                                  sp.turretVelocityRadsPerSec);
                              Logger.recordOutput(
                                  "shooterSetpoint/turretAccelerationRadsPerSecSquared",
                                  sp.turretAccelerationRadsPerSecSquared);
                              Logger.recordOutput(
                                  "shooterSetpoint/hoodPositionRadians", sp.hoodPositionRadians);
                              Logger.recordOutput(
                                  "shooterSetpoint/hoodVelocityRadsPerSec",
                                  sp.hoodVelocityRadsPerSec);
                              Logger.recordOutput(
                                  "shooterSetpoint/hoodAccelerationRadsPerSecSquared",
                                  sp.hoodAccelerationRadsPerSecSquared);
                            },
                            triggers,
                            indexer))),
            Map.entry(
                SuperstructureState.SHOOTING_FIXED,
                () ->
                    Commands.parallel(
                        turret.setGoalCommand(TurretGoal.FIXED_ANGLE),
                        hood.setGoalCommand(HoodGoal.FIXED_ANGLE),
                        flywheel.setGoalCommand(FlywheelGoal.FIXED_VELOCITY),
                        Commands.run(
                            () -> {
                              if (isReadyToShoot()) {
                                triggers.setGoal(TriggersGoal.SHOOT);
                                indexer.setGoal(IndexerGoal.SHOOT);
                              } else {
                                triggers.setGoal(TriggersGoal.STOP);
                                indexer.setGoal(IndexerGoal.STOP);
                              }
                            },
                            triggers,
                            indexer))),
            Map.entry(
                SuperstructureState.PASSING,
                () ->
                    Commands.parallel(
                        turret.setGoalCommand(TurretGoal.PASSING),
                        hood.setGoalCommand(HoodGoal.PASSING),
                        flywheel.setGoalCommand(FlywheelGoal.PASSING),
                        Commands.run(
                            () -> {
                              Translation2d targetPos =
                                  getBestPassingTarget(TurretConstants.swerve2TurretStructure);
                              ShooterSetpoint sp =
                                  ShooterSetpoint.makeSetpoint(
                                      TurretConstants.swerve2TurretStructure,
                                      targetPos,
                                      FieldConstants.hMax,
                                      HoodConstants.kHoodMinAngle,
                                      HoodConstants.kHoodMaxAngle,
                                      TrajectoryConfig.getHubConfig());

                              turret.runPositionFOCLogic(
                                  sp.turretPositionRadians,
                                  sp.turretVelocityRadsPerSec,
                                  sp.turretAccelerationRadsPerSecSquared,
                                  // 0,
                                  // 0,
                                  0.0);
                              hood.runPositionFOCLogic(
                                  sp.hoodPositionRadians,
                                  sp.hoodVelocityRadsPerSec,
                                  sp.hoodAccelerationRadsPerSecSquared,
                                  0.0);

                              double radPerSec =
                                  TrajectoryCalculator.getFlywheelSetpoint(
                                          sp.shooterVelocityMetersPerSec)
                                      / FlywheelConstants.kFlywheelRadius;
                              double radPerSec2 =
                                  TrajectoryCalculator.getFlywheelAcceleration(
                                          sp.shooterVelocityMetersPerSec,
                                          sp.shooterAccelerationMetersPerSecSquared)
                                      / FlywheelConstants.kFlywheelRadius;
                              flywheel.runVelocityFOCLogic(radPerSec, radPerSec2, 0.0);

                              if (isReadyToShoot()) {
                                triggers.setGoal(TriggersGoal.SHOOT);
                                indexer.setGoal(IndexerGoal.SHOOT);
                              } else {
                                triggers.setGoal(TriggersGoal.STOP);
                                indexer.setGoal(IndexerGoal.STOP);
                              }
                            },
                            triggers,
                            indexer))),
            Map.entry(
                SuperstructureState.TRENCH,
                () ->
                    Commands.parallel(
                        turret.setGoalCommand(TurretGoal.IDLE),
                        hood.setGoalCommand(HoodGoal.ZEROING),
                        flywheel.setGoalCommand(FlywheelGoal.TRACKING),
                        Commands.runOnce(
                            () -> {
                              triggers.setGoal(TriggersGoal.STOP);
                              indexer.setGoal(IndexerGoal.STOP);
                            },
                            triggers,
                            indexer))),
            Map.entry(
                SuperstructureState.TEST,
                () ->
                    Commands.parallel(
                        turret.setGoalCommand(TurretGoal.TEST),
                        // hood.setGoalCommand(HoodGoal.TEST),
                        // flywheel.setGoalCommand(FlywheelGoal.TEST),
                        Commands.run(
                            () -> {
                              if (isReadyToShoot()) {
                                triggers.setGoal(TriggersGoal.SHOOT);
                                indexer.setGoal(IndexerGoal.SHOOT);
                              } else {
                                triggers.setGoal(TriggersGoal.STOP);
                                indexer.setGoal(IndexerGoal.STOP);
                              }
                            },
                            triggers,
                            indexer))));
  }
  /** 检查是否准备好发射 */
  @AutoLogOutput(key = "Superstructure/ReadyToShoot")
  public boolean isReadyToShoot() {
    return flywheel.atGoal() && hood.atGoal() && turret.atGoal();
  }

  @AutoLogOutput(key = "Superstructure/GoalCommand")
  public Command setGoal(SuperstructureState requestedState) {
    return Commands.defer(
        () -> {
          // 1. 如果当前处于 Teleop 且 在 Trench 区域内
          if (DriverStation.isTeleop() && inTrenchZoneSupplier.getAsBoolean()) {
            if (requestedState != SuperstructureState.TRENCH) {
              this.fallbackState = requestedState;
              // 不执行请求，返回一个空的立刻结束的 Command
              return Commands.none();
            }
          }

          // 2. 正常情况：更新 fallback 并执行真实状态
          return Commands.sequence(
              Commands.runOnce(
                  () -> {
                    this.currentState = requestedState;
                    this.fallbackState = requestedState;
                  }),
              stateMap.get(requestedState).get());
        },
        java.util.Set.of(this));
  }

  @Override
  public void periodic() {
    handleSafetyOverride();
    updateLEDState();
  }

  private void handleSafetyOverride() {
    // 只有在 Teleop 且在 Trench 区域内才视为危险情况
    boolean inDangerZone = DriverStation.isTeleop() && inTrenchZoneSupplier.getAsBoolean();

    if (inDangerZone) {
      // 刚进入危险区：如果当前状态不是 TRENCH，立刻强行覆盖！
      if (currentState != SuperstructureState.TRENCH) {
        // 记录被打断前的状态，以便后续恢复
        fallbackState = currentState;
        currentState = SuperstructureState.TRENCH;
        // 【安全强制越权】：直接修改子系统状态，绕过 Command 队列的延迟
        forceTrenchMode();
      }
    } else {
      // 离开危险区：如果当前状态是因为安全机制被卡在 TRENCH 的
      if (currentState == SuperstructureState.TRENCH
          && fallbackState != SuperstructureState.TRENCH) {
        // 自动恢复到进入隧道前原本想要执行的状态！
        SuperstructureState stateToRestore = fallbackState;
        // 为了防止死循环恢复，先重置 fallback，再执行恢复命令
        fallbackState = SuperstructureState.TRENCH;
        // 调度原有的 Command 进行平滑过渡
        setGoal(stateToRestore).schedule();
      }
    }
    Logger.recordOutput("Superstructure/inDangerZone", inDangerZone);
  }

  private void forceTrenchMode() {
    hood.setGoal(HoodGoal.ZEROING); // 强行压低
  }

  private void updateLEDState() {
    // --- 0. 自动阶段 (强制最高优先级) ---
    if (DriverStation.isAutonomous()) {
      led.setGoal(LED.LEDState.AUTO);
      return;
    }

    // --- 1. TRENCH 保护 (物理安全，最高优先级) ---
    if (currentState == SuperstructureState.TRENCH) {
      led.setGoal(LED.LEDState.TRENCH);
      return;
    }

    // --- 2. 发射与瞄准 (高频操作任务) ---
    if (currentState == SuperstructureState.ACTIVESHOOTING
        || currentState == SuperstructureState.SHOOTING
        || currentState == SuperstructureState.SHOOTING_FIXED
        || currentState == SuperstructureState.PASSING) {

      if (isReadyToShoot()) {
        if (currentState == SuperstructureState.SHOOTING) {
          led.setGoal(LED.LEDState.SHOOTING);
        } else if (currentState == SuperstructureState.PASSING) {
          led.setGoal(LED.LEDState.PASSING);
        } else {
          led.setGoal(LED.LEDState.READY_TO_SHOOT);
        }
      } else {
        led.setGoal(LED.LEDState.INITIAL);
      }
      return; // 执行了射击逻辑，直接返回
    }

    // --- 3. 吸球动作 (主动任务) ---
    if (intake.getGoal() == Intake.IntakeGoal.INTAKE) {
      led.setGoal(LED.LEDState.INTAKING);
      return;
    }
    // --- 5. STOW (收起状态 - 被动) ---
    if (intake.getGoal() == Intake.IntakeGoal.STOW) {
      led.setGoal(LED.LEDState.INTAKE_STOWED);
      return;
    }
    // --- 6. 默认状态 (IDLE / 联盟色) ---
    led.setGoal(LED.LEDState.INITIAL);
  }

  // TODO move it to where it belong
  private Translation2d getBestPassingTarget(PhysicalJoint base) {
    Translation2d blueLeft = new Translation2d(1.874, 5.49);
    Translation2d blueRight = new Translation2d(1.874, 2.17);
    Translation2d left = AllianceFlipUtil.apply(blueLeft);
    Translation2d right = AllianceFlipUtil.apply(blueRight);

    // 从物理关节获取当前机器人在场地的位置
    Translation2d robotPos = base.getGlobalPose().getTranslation().toTranslation2d();
    return (robotPos.getDistance(left) < robotPos.getDistance(right)) ? left : right;
  }
}
