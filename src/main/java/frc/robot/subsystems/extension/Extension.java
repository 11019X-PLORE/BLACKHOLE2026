package frc.robot.subsystems.extension;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.subsystems.extension.ExtensionIO.ExtensionIOOutputMode;
import frc.robot.subsystems.extension.ExtensionIO.ExtensionIOOutputs;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Extension extends FullSubsystem {
  // --- Tunables ---
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Extension/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Extension/kD");
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Extension/kI");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Extension/kS");
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Extension/kV");
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Extension/kA");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Extension/kG");
  private static final LoggedTunableNumber toleranceDeg =
      new LoggedTunableNumber("Extension/ToleranceDeg");

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
    } else {
      kP.initDefault(ExtensionConstants.kP);
      kI.initDefault(ExtensionConstants.kI);
      kD.initDefault(ExtensionConstants.kD);
      kS.initDefault(ExtensionConstants.kS);
      kV.initDefault(ExtensionConstants.kV);
      kA.initDefault(ExtensionConstants.kA);
      kG.initDefault(ExtensionConstants.kG);
      toleranceDeg.initDefault(ExtensionConstants.kExtensiontoleranceDeg);
    }
  }

  // --- IO & Inputs ---
  private final ExtensionIO io;
  private final ExtensionIOInputsAutoLogged inputs = new ExtensionIOInputsAutoLogged();
  private final ExtensionIOOutputs outputs = new ExtensionIOOutputs();

  // --- State Variables (Goal-based State Machine) ---
  public enum ExtensionGoal {
    IDLE, // 待机/刹车 (不施加闭环控制)
    STOWED, // 收起状态 (闭环维持在初始安全位置)
    DEPLOYED, // 展开吸球状态 (闭环维持在吸球角度)
    FIXED_ANGLE, // 任意指定角度 (供测试或特殊位置使用)
    ZEROING,
    SHAKE // 归零状态
  }

  @Getter @Setter @AutoLogOutput private ExtensionGoal goal = ExtensionGoal.IDLE;

  @Getter @AutoLogOutput private boolean ExtensionZeroed = false;

  @Getter @AutoLogOutput private boolean atGoal = false;

  // --- Hardware Safety & Alerts ---
  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert motorDisconnectedAlert =
      new Alert("Extension motor disconnected!", Alert.AlertType.kWarning);

  public Extension(ExtensionIO io) {
    this.io = io;
    io.setPID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get(), kG.get());
  }

  @Override
  public void periodic() {
    // 1. 读取输入
    io.updateInputs(inputs);
    Logger.processInputs("Extension", inputs);

    // 2. 更新硬件报警与 Tunables
    motorDisconnectedAlert.set(!motorConnectedDebouncer.calculate(inputs.motorConnected));
    updateTunables();
  }

  @Override
  public void periodicAfterScheduler() {
    if (DriverStation.isDisabled() || !ExtensionZeroed) {
      outputs.mode = ExtensionIOOutputMode.COAST;
      outputs.velocityRadsPerSec = 0.0;
      atGoal = false;
    } else {
      switch (goal) {
        case IDLE -> {
          outputs.mode = ExtensionIOOutputMode.COAST;
          atGoal = true;
        }
        case STOWED -> {
          runPositionLogic(ExtensionConstants.kExtensionStowerAngle);
        }
        case DEPLOYED -> {
          runPositionLogic(ExtensionConstants.kExtensionDeployAngle);
        }
        case FIXED_ANGLE -> {
          runPositionLogic(ExtensionConstants.kFixAngle);
        }
        case ZEROING -> {
          outputs.mode = ExtensionIOOutputMode.COAST;
          outputs.velocityRadsPerSec = 0.0;
        }
        case SHAKE -> {
          // 利用时间戳计算当前的往复目标
          // 每隔 (1/SHAKE_FREQUENCY)/2 秒切换一次方向
          double time = Timer.getFPGATimestamp();
          boolean isHigh = (int) (time * ExtensionConstants.kShakeFrequency * 2) % 2 == 0;

          // 在 0° (Deploy) 和 90° (Stow) 之间切换
          double shakeTarget =
              isHigh
                  ? ExtensionConstants.kExtensionStowerAngle
                  : ExtensionConstants.kExtensionShakeAngle;

          // 执行运动（为了摇晃更有力，建议使用较大的速度限制）
          runPositionLogic(shakeTarget);
        }
      }
    }
  }

  /** 内部位置闭环辅助方法：负责 clamp 角度、设置 output 并计算 atGoal */
  private void runPositionLogic(double targetAngleRads) {
    double clampedAngle =
        MathUtil.clamp(
            targetAngleRads,
            ExtensionConstants.kExtensionMinAngle,
            ExtensionConstants.kExtensionMaxAngle);

    outputs.mode = ExtensionIOOutputMode.CLOSED_LOOP;
    outputs.positionRads = clampedAngle;

    // 计算是否到位
    atGoal = Math.abs(inputs.positionRads - clampedAngle) <= toleranceDeg.get();

    // Log 目标值
    Logger.recordOutput("Extension/Profile/GoalPositionRad", clampedAngle);
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
  public Command setGoalCommand(ExtensionGoal newGoal) {
    return Commands.runOnce(() -> this.goal = newGoal, this);
  }

  /** 归零命令 */
  public Command zeroCommand() {
    return runOnce(
            () -> {
              ExtensionZeroed = true;
              this.goal = ExtensionGoal.IDLE;
            })
        .ignoringDisable(true);
  }
}
