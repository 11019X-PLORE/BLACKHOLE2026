package frc.robot.subsystems.intakearm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.intakearm.IntakearmIO.IntakearmIOOutputMode;
import frc.robot.subsystems.intakearm.IntakearmIO.IntakearmIOOutputs;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intakearm extends SubsystemBase {
  // --- Tunables ---
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Intakearm/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Intakearm/kD");
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Intakearm/kI");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Intakearm/kS");
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Intakearm/kV");
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Intakearm/kA");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Intakearm/kG");
  private static final LoggedTunableNumber toleranceDeg =
      new LoggedTunableNumber("Intakearm/ToleranceDeg");
  private static final LoggedTunableNumber kHoldVoltage =
      new LoggedTunableNumber("Intakearm/HoldVoltage");

  static {
    // 真实虚拟机调试参数不同
    if (Robot.isSimulation()) {
      kP.initDefault(10.0);
      kI.initDefault(0.0);
      kD.initDefault(0.1);
      kS.initDefault(0.0);
      kV.initDefault(0.0);
      kA.initDefault(0.0);
      kG.initDefault(0.0);
      toleranceDeg.initDefault(0.01);
      kHoldVoltage.initDefault(-1.0);
    } else {
      kP.initDefault(IntakearmConstants.kP);
      kI.initDefault(IntakearmConstants.kI);
      kD.initDefault(IntakearmConstants.kD);
      kS.initDefault(IntakearmConstants.kS);
      kV.initDefault(IntakearmConstants.kV);
      kA.initDefault(IntakearmConstants.kA);
      kG.initDefault(IntakearmConstants.kG);
      toleranceDeg.initDefault(IntakearmConstants.kIntakearmtoleranceDeg);
      kHoldVoltage.initDefault(IntakearmConstants.kHoldVoltage);
    }
  }

  // --- IO & Inputs ---
  private final IntakearmIO io;
  private final IntakearmIOInputsAutoLogged inputs = new IntakearmIOInputsAutoLogged();
  private final IntakearmIOOutputs outputs = new IntakearmIOOutputs();

  // --- State Variables (Goal-based State Machine) ---
  public enum IntakearmGoal {
    IDLE, // 待机/刹车 (不施加闭环控制)
    STOWED, // 收起状态 (闭环维持在初始安全位置)
    DEPLOYED, // 展开吸球状态 (闭环维持在吸球角度)
    FIXED_ANGLE, // 任意指定角度 (供测试或特殊位置使用)
    ZEROING,
    SHAKE // 归零状态
  }

  @Getter @Setter @AutoLogOutput private IntakearmGoal goal = IntakearmGoal.IDLE;

  // 供 FIXED_ANGLE 模式使用
  @Setter private double fixedAngleRads = IntakearmConstants.kFixAngle;

  @Getter @AutoLogOutput private boolean intakearmZeroed = false;

  @Getter @AutoLogOutput private boolean atGoal = false;
  @Getter @AutoLogOutput private boolean hasZeroedAtPosition = false;
  @Getter @AutoLogOutput private boolean isDeployedLocked = false;

  // --- Hardware Safety & Alerts ---
  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert motorDisconnectedAlert =
      new Alert("Intakearm motor disconnected!", Alert.AlertType.kWarning);
  private double currentTargetAngle = IntakearmConstants.kIntakearmDeployAngle;

  public Intakearm(IntakearmIO io) {
    this.io = io;
    io.setPID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get(), kG.get());
  }

  @Override
  public void periodic() {
    // 1. 读取输入
    io.updateInputs(inputs);
    Logger.processInputs("Intakearm", inputs);

    // 2. 更新硬件报警与 Tunables
    motorDisconnectedAlert.set(!motorConnectedDebouncer.calculate(inputs.motorConnected));
    updateTunables();

    if (DriverStation.isDisabled() || !intakearmZeroed) {
      outputs.mode = IntakearmIOOutputMode.COAST;
      outputs.velocityRadsPerSec = 0.0;
      atGoal = false;
      isDeployedLocked = false;
    } else {
      switch (goal) {
        case IDLE -> {
          outputs.mode = IntakearmIOOutputMode.COAST;
          atGoal = true;
          isDeployedLocked = false;
        }
        case STOWED -> {
          runPositionLogic(
              IntakearmConstants.kIntakearmStowerAngle, IntakearmConstants.kVelocityRadPerSec);
          isDeployedLocked = false;
        }
        case DEPLOYED -> {
          double target = IntakearmConstants.kIntakearmDeployAngle;

          if (isDeployedLocked) {
            outputs.mode = IntakearmIOOutputMode.VOLTAGE;
            outputs.appliedVolts = kHoldVoltage.get();
            atGoal = true;
          } else {
            boolean nearTarget = Math.abs(inputs.positionRads - target) <= toleranceDeg.get();

            if (!nearTarget) {
              runPositionLogic(target, IntakearmConstants.kVelocityRadPerSec);
            } else {
              isDeployedLocked = true;
              atGoal = true;
              outputs.mode = IntakearmIOOutputMode.VOLTAGE;
              outputs.appliedVolts = kHoldVoltage.get();
            }
          }
        }
        case FIXED_ANGLE -> {
          runPositionLogic(fixedAngleRads, IntakearmConstants.kVelocityRadPerSec);
          isDeployedLocked = false;
        }
        case ZEROING -> {
          outputs.mode = IntakearmIOOutputMode.COAST;
          outputs.velocityRadsPerSec = 0.0;
          isDeployedLocked = false;
        }
        case SHAKE -> {
          isDeployedLocked = false; // 清除锁定状态

          // 利用时间戳计算当前的往复目标
          // 每隔 (1/SHAKE_FREQUENCY)/2 秒切换一次方向
          double time = Timer.getFPGATimestamp();
          boolean isHigh = (int) (time * IntakearmConstants.kShakeFrequency * 2) % 2 == 0;

          // 在 0° (Deploy) 和 90° (Stow) 之间切换
          double shakeTarget =
              isHigh
                  ? IntakearmConstants.kIntakearmStowerAngle
                  : IntakearmConstants.kIntakearmShakeAngle;

          // 执行运动（为了摇晃更有力，建议使用较大的速度限制）
          runPositionLogic(shakeTarget, IntakearmConstants.kVelocityRadPerSec);
        }
      }
    }

    // 4. 应用输出
    Logger.recordOutput("Intakearm/Profile/mode", outputs.mode);
    io.applyOutputs(outputs);
  }

  /** 内部位置闭环辅助方法：负责 clamp 角度、设置 output 并计算 atGoal */
  private void runPositionLogic(double targetAngleRads, double targetVelocityRadsPerSec) {
    double clampedAngle =
        MathUtil.clamp(
            targetAngleRads,
            IntakearmConstants.kIntakearmMinAngle,
            IntakearmConstants.kIntakearmMaxAngle);

    outputs.mode = IntakearmIOOutputMode.CLOSED_LOOP;
    outputs.positionRad = clampedAngle;
    outputs.velocityRadsPerSec = targetVelocityRadsPerSec;

    // 计算是否到位
    atGoal = Math.abs(inputs.positionRads - clampedAngle) <= toleranceDeg.get();

    // Log 目标值
    Logger.recordOutput("Intakearm/Profile/GoalPositionRad", clampedAngle);
  }

  /** 更新 PID 参数 */
  private void updateTunables() {
    if (kP.hasChanged(hashCode())
        || kI.hasChanged(hashCode())
        || kD.hasChanged(hashCode())
        || kS.hasChanged(hashCode())
        || kV.hasChanged(hashCode())
        || kA.hasChanged(hashCode())
        || kG.hasChanged(hashCode())) {
      io.setPID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get(), kG.get());
    }
  }

  public double getMeasuredAngleRad() {
    return inputs.positionRads;
  }

  // --- Commands (供 Superstructure 调用) ---
  public Command setGoalCommand(IntakearmGoal newGoal) {
    return Commands.runOnce(() -> this.goal = newGoal, this);
  }

  /** 归零命令 */
  public Command zeroCommand() {
    return runOnce(
            () -> {
              intakearmZeroed = true;
              this.goal = IntakearmGoal.IDLE;
            })
        .ignoringDisable(true);
  }
}
