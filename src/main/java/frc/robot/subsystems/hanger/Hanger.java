package frc.robot.subsystems.hanger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.hanger.HangerIO.HangerIOOutputMode;
import frc.robot.subsystems.hanger.HangerIO.HangerIOOutputs;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hanger extends SubsystemBase {
  // --- Tunables ---
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Hanger/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Hanger/kD");
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Hanger/kI");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Hanger/kS");
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Hanger/kV");
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Hanger/kA");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Hanger/kG");
  private static final LoggedTunableNumber toleranceDeg =
      new LoggedTunableNumber("Hanger/ToleranceDeg");

  static {
    // 真实与模拟器调试参数
    if (Robot.isSimulation()) {
      kP.initDefault(0.035);
      kI.initDefault(0.0);
      kD.initDefault(0.1);
      kS.initDefault(0.0);
      kV.initDefault(0.0);
      kA.initDefault(0.0);
      kG.initDefault(0.0);
      toleranceDeg.initDefault(1.0);
    } else {
      kP.initDefault(HangerConstants.kP);
      kI.initDefault(HangerConstants.kI);
      kD.initDefault(HangerConstants.kD);
      kS.initDefault(HangerConstants.kS);
      kV.initDefault(HangerConstants.kV);
      kA.initDefault(HangerConstants.kA);
      kG.initDefault(HangerConstants.kG);
      toleranceDeg.initDefault(HangerConstants.kHangertoleranceDeg);
    }
  }

  // --- IO & Inputs ---
  private final HangerIO io;
  private final HangerIOInputsAutoLogged inputs = new HangerIOInputsAutoLogged();
  private final HangerIOOutputs outputs = new HangerIOOutputs();
  private final Timer stallTimer = new Timer();

  // --- State Variables (Goal-based State Machine) ---
  public enum HangerGoal {
    IDLE, // 待机/刹车，不施加闭环控制
    STOWED, // 初始收起位置 (底部)
    EXTENDED, // 完全伸出位置 (准备挂钩)
    CLIMBING, // 爬升收缩位置 (将机器人拉起)
    FIXED_ANGLE, // 指定固定角度 (供测试或特殊微调使用)
    ZEROING // 归零状态
  }

  @Getter @Setter @AutoLogOutput private HangerGoal goal = HangerGoal.IDLE;

  // 供 FIXED_ANGLE 模式使用
  @Setter private double fixedAngleRads = HangerConstants.kHangerInitialAngle;

  @Getter @AutoLogOutput private boolean hangerZeroed = false;

  @Getter @AutoLogOutput private boolean atGoal = false;

  // --- Hardware Safety & Alerts ---
  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert motorDisconnectedAlert =
      new Alert("Hanger motor disconnected!", Alert.AlertType.kWarning);

  public Hanger(HangerIO io) {
    this.io = io;
    io.setPID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get(), kG.get());
    stallTimer.start();
  }

  @Override
  public void periodic() {
    // 1. 读取输入
    io.updateInputs(inputs);
    Logger.processInputs("Hanger", inputs);

    // 2. 更新硬件报警与 Tunables
    motorDisconnectedAlert.set(!motorConnectedDebouncer.calculate(inputs.motorConnected));
    updateTunables();

    // 3. 核心控制循环 (状态机)
    // 安全机制：如果未开启或未归零，强制刹车 (除非正在归零)
    if (DriverStation.isDisabled() || !hangerZeroed) {
      outputs.mode = HangerIOOutputMode.BRAKE;
      outputs.velocityRadsPerSec = 0.0;
      atGoal = false;
    } else {
      switch (goal) {
        case IDLE -> {
          // 爬升机构通常需要强力的 BRAKE 来保持机器人在空中
          outputs.mode = HangerIOOutputMode.BRAKE;
          outputs.velocityRadsPerSec = 0.0;
          atGoal = true;
        }
        case STOWED -> {
          // 运行到初始收起位置
          runPositionLogic(HangerConstants.kHangerInitialAngle);
        }
        case EXTENDED -> {
          // 运行到最大伸出位置 (举例：用于勾住横杆)
          runPositionLogic(HangerConstants.kHangerMaxAngle);
        }
        case CLIMBING -> {
          // 运行到收缩拉起机器人的位置
          runPositionLogic(HangerConstants.kHangerClimbAngle);
        }
        case FIXED_ANGLE -> {
          // 运行到手动指定的任意角度
          runPositionLogic(fixedAngleRads);
        }
        case ZEROING -> {
          runZeroingLogic();
        }
      }
    }

    // 4. 应用输出并记录日志
    Logger.recordOutput("Hanger/Profile/mode", outputs.mode);
    io.applyOutputs(outputs);
  }

  /** 内部位置闭环辅助方法：负责 clamp 角度、设置 output 并计算 atGoal */
  private void runPositionLogic(double targetAngleRads) {
    // 软限位保护
    double clampedAngle =
        MathUtil.clamp(
            targetAngleRads, HangerConstants.kHangerMinAngle, HangerConstants.kHangerMaxAngle);

    outputs.mode = HangerIOOutputMode.CLOSED_LOOP;
    outputs.positionRad = clampedAngle;

    // 计算是否到位
    atGoal =
        Math.abs(inputs.positionRads - clampedAngle) <= Units.degreesToRadians(toleranceDeg.get());

    // 记录目标日志
    Logger.recordOutput("Hanger/Profile/GoalPositionRad", clampedAngle);
  }

  /** 归零核心逻辑： 施加电压 -> 滤除启动尖峰 -> 检测堵转 -> 归零 -> 切回 IDLE */
  private void runZeroingLogic() {
    outputs.mode = HangerIOOutputMode.VOLTAGE;
    outputs.appliedVolts = HangerConstants.kHangerStallVolts;

    if (Math.abs(inputs.torqueCurrentAmps) > HangerConstants.kHangerStallTorqueCurrent
        && Math.abs(inputs.velocityRadsPerSec) < HangerConstants.kHangerStallVelocity) {
      // 正在堵转，计时器继续跑
    } else {
      // 没堵转 (或者刚启动电流还没上来)，重置计时器
      stallTimer.reset();
    }

    // 4. 判定完成：必须持续堵转 0.2s 才能确认到底了
    if (stallTimer.hasElapsed(HangerConstants.kHangerStallTime)) {
      io.resetPosition(HangerConstants.kHangerInitialAngle); // 硬件归零
      hangerZeroed = true; // 软件标记
      goal = HangerGoal.IDLE; // 任务完成，自动切回待机

      // 立即刹车，防止继续推
      outputs.mode = HangerIOOutputMode.BRAKE;
      outputs.velocityRadsPerSec = 0.0;
    }
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

  // --- Commands (直接供 RobotContainer 调用) ---
  public Command setGoalCommand(HangerGoal newGoal) {
    return Commands.runOnce(() -> this.goal = newGoal);
  }

  public Command runFixedAngleCommand(double targetRads) {
    return Commands.sequence(
        Commands.runOnce(() -> this.fixedAngleRads = targetRads),
        setGoalCommand(HangerGoal.FIXED_ANGLE));
  }

  /** 归零命令 */
  public Command zeroCommand() {
    return runOnce(
            () -> {
              hangerZeroed = true;
              this.goal = HangerGoal.IDLE;
            })
        .ignoringDisable(true);
  }
}
