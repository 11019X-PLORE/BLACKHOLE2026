package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hanger.Hanger;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerGoal;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeGoal;
import frc.robot.subsystems.intakearm.Intakearm;
import frc.robot.subsystems.intakearm.Intakearm.IntakearmGoal;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.rotator.Rotator;
import frc.robot.subsystems.rotator.Rotator.RotatorGoal;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.Flywheel.FlywheelGoal;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.Hood.HoodGoal;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.Turret.TurretGoal;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {

  private final Turret turret;
  private final Hood hood;
  private final Flywheel flywheel;
  private final Intake intake;
  private final Intakearm intakearm;
  private final Rotator rotator;
  private final Indexer indexer;
  private final Hanger hanger;
  private final LED led;

  public enum SuperstructureState {
    IDLE,
    INTAKE,
    SPIT,
    SHOOTSPIT,
    STOW,
    INTAKESTOP,
    SPITSTOP,
    ACTIVESHOOTING,
    SHOOTING,
    SHOOTING_FIXED,
    PASSING,
    TRENCH,
    TEST,
    SHAKE,
  }

  @Getter @AutoLogOutput private SuperstructureState currentState = SuperstructureState.IDLE;
  @AutoLogOutput private SuperstructureState fallbackState = SuperstructureState.IDLE;

  private final Map<SuperstructureState, Supplier<Command>> stateMap;
  private final BooleanSupplier inTrenchZoneSupplier;

  public Superstructure(
      Turret turret,
      Hood hood,
      Flywheel flywheel,
      Intake intake,
      Intakearm intakearm,
      Rotator rotator,
      Indexer indexer,
      Hanger hanger,
      LED led,
      BooleanSupplier inTrenchZoneSupplier) {
    this.turret = turret;
    this.hood = hood;
    this.flywheel = flywheel;
    this.intake = intake;
    this.intakearm = intakearm;
    this.rotator = rotator;
    this.indexer = indexer;
    this.hanger = hanger;
    this.led = led;
    this.inTrenchZoneSupplier = inTrenchZoneSupplier;

    stateMap =
        Map.ofEntries(
            Map.entry(
                SuperstructureState.IDLE,
                () ->
                    Commands.parallel(
                        turret.setGoalCommand(TurretGoal.IDLE),
                        hood.setGoalCommand(HoodGoal.IDLE),
                        flywheel.setGoalCommand(FlywheelGoal.IDLE),
                        intakearm.setGoalCommand(IntakearmGoal.IDLE),
                        intake.setGoalCommand(IntakeGoal.STOP),
                        Commands.runOnce(
                            () -> {
                              rotator.setGoal(RotatorGoal.STOP);
                              indexer.setGoal(IndexerGoal.STOP);
                            },
                            rotator,
                            indexer))),
            Map.entry(
                SuperstructureState.INTAKE,
                () ->
                    Commands.parallel(
                        intakearm.setGoalCommand(IntakearmGoal.DEPLOYED),
                        intake.setGoalCommand(IntakeGoal.INTAKE))),
            Map.entry(
                SuperstructureState.SPIT,
                () ->
                    Commands.parallel(
                        intakearm.setGoalCommand(IntakearmGoal.DEPLOYED),
                        intake.setGoalCommand(IntakeGoal.OUTTAKE),
                        Commands.run(
                            () -> {
                              rotator.setGoal(RotatorGoal.OUTTAKE);
                              indexer.setGoal(IndexerGoal.OUTTAKE);
                            },
                            rotator,
                            indexer))),
            Map.entry(
                SuperstructureState.SPITSTOP,
                () ->
                    Commands.parallel(
                        intakearm.setGoalCommand(IntakearmGoal.DEPLOYED),
                        intake.setGoalCommand(IntakeGoal.STOP),
                        Commands.run(
                            () -> {
                              rotator.setGoal(RotatorGoal.STOP);
                              indexer.setGoal(IndexerGoal.STOP);
                            },
                            rotator,
                            indexer))),
            Map.entry(
                SuperstructureState.SHOOTSPIT,
                () ->
                    Commands.parallel(
                        flywheel.setGoalCommand(FlywheelGoal.FIXED_VELOCITY),
                        Commands.runOnce(
                            () -> {
                              rotator.setGoal(RotatorGoal.SHOOT);
                              indexer.setGoal(IndexerGoal.SHOOT);
                            },
                            rotator,
                            indexer))),
            Map.entry(
                SuperstructureState.INTAKESTOP,
                () ->
                    Commands.parallel(
                        intakearm.setGoalCommand(IntakearmGoal.DEPLOYED),
                        intake.setGoalCommand(IntakeGoal.STOP))),
            Map.entry(
                SuperstructureState.STOW,
                () ->
                    Commands.parallel(
                        intakearm.setGoalCommand(IntakearmGoal.STOWED),
                        intake.setGoalCommand(IntakeGoal.STOP))),
            Map.entry(
                SuperstructureState.SHAKE,
                () ->
                    Commands.parallel(
                        intakearm.setGoalCommand(IntakearmGoal.SHAKE),
                        intake.setGoalCommand(IntakeGoal.STOW))),
            Map.entry(
                SuperstructureState.ACTIVESHOOTING,
                () ->
                    Commands.parallel(
                        turret.setGoalCommand(TurretGoal.ZEROING),
                        hood.setGoalCommand(HoodGoal.ZEROING),
                        flywheel.setGoalCommand(FlywheelGoal.ACTIVE),
                        Commands.run(
                            () -> {
                              rotator.setGoal(RotatorGoal.STOP);
                              indexer.setGoal(IndexerGoal.STOP);
                            },
                            rotator,
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
                              if (isReadyToShoot()) {
                                rotator.setGoal(RotatorGoal.SHOOT);
                                indexer.setGoal(IndexerGoal.SHOOT);
                              } else {
                                rotator.setGoal(RotatorGoal.STOP);
                                indexer.setGoal(IndexerGoal.STOP);
                              }
                            },
                            rotator,
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
                                rotator.setGoal(RotatorGoal.SHOOT);
                                indexer.setGoal(IndexerGoal.SHOOT);
                              } else {
                                rotator.setGoal(RotatorGoal.STOP);
                                indexer.setGoal(IndexerGoal.STOP);
                              }
                            },
                            rotator,
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
                              if (isReadyToShoot()) {
                                indexer.setGoal(IndexerGoal.SHOOT);
                                rotator.setGoal(RotatorGoal.SHOOT);
                              } else {
                                indexer.setGoal(IndexerGoal.STOP);
                                rotator.setGoal(RotatorGoal.STOP);
                              }
                            },
                            rotator,
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
                              rotator.setGoal(RotatorGoal.STOP);
                              indexer.setGoal(IndexerGoal.STOP);
                            },
                            rotator,
                            indexer))),
            Map.entry(
                SuperstructureState.TEST,
                () ->
                    Commands.parallel(
                        turret.setGoalCommand(TurretGoal.IDLE),
                        hood.setGoalCommand(HoodGoal.TEST),
                        flywheel.setGoalCommand(FlywheelGoal.TEST),
                        Commands.run(
                            () -> {
                              if (isReadyToShoot()) {
                                rotator.setGoal(RotatorGoal.SHOOT);
                                indexer.setGoal(IndexerGoal.SHOOT);
                              } else {
                                rotator.setGoal(RotatorGoal.STOP);
                                indexer.setGoal(IndexerGoal.STOP);
                              }
                            },
                            rotator,
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
    if (currentState == SuperstructureState.INTAKE) {
      led.setGoal(LED.LEDState.INTAKING);
      return;
    }

    // --- 4. 爬升状态 (主动任务) ---
    // 将爬升放在 STOW 之前，确保即使在收起位，只要爬升架伸出去了，就显示彩虹灯
    if (hanger.getGoal() == Hanger.HangerGoal.EXTENDED
        || hanger.getGoal() == Hanger.HangerGoal.CLIMBING) {
      led.setGoal(LED.LEDState.CLIMBING);
      return;
    }

    // --- 5. STOW (收起状态 - 被动) ---
    if (currentState == SuperstructureState.STOW) {
      led.setGoal(LED.LEDState.INTAKE_STOWED);
      return;
    }
    // --- 6. 默认状态 (IDLE / 联盟色) ---
    led.setGoal(LED.LEDState.INITIAL);
  }
}
