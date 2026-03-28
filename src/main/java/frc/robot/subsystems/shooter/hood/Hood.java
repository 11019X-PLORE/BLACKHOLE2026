package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.hood.HoodIO.HoodIOOutputMode;
import frc.robot.subsystems.shooter.hood.HoodIO.HoodIOOutputs;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {

  // --- Tunables ---
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Hood/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Hood/kD");
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Hood/kI");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Hood/kS");
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Hood/kV");
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Hood/kA");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Hood/kG");
  private static final LoggedTunableNumber kFixAngle = new LoggedTunableNumber("Hood/kFixAngle");
  private static final LoggedTunableNumber toleranceDeg =
      new LoggedTunableNumber("Hood/ToleranceDeg");

  static {
    if (Robot.isSimulation()) {
      kP.initDefault(0.035);
      kI.initDefault(0.0);
      kD.initDefault(0.1);
      kS.initDefault(0.0);
      kV.initDefault(0.0);
      kA.initDefault(0.0);
      kG.initDefault(0.0);
      toleranceDeg.initDefault(1.0);
      kFixAngle.initDefault(10);
    } else {
      kP.initDefault(HoodConstants.kP);
      kI.initDefault(HoodConstants.kI);
      kD.initDefault(HoodConstants.kD);
      kS.initDefault(HoodConstants.kS);
      kV.initDefault(HoodConstants.kV);
      kA.initDefault(HoodConstants.kA);
      kG.initDefault(HoodConstants.kG);
      toleranceDeg.initDefault(HoodConstants.kHoodtoleranceDeg);
      kFixAngle.initDefault(HoodConstants.kFixAngle);
    }
  }

  // --- IO & Inputs ---
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final HoodIOOutputs outputs = new HoodIOOutputs();

  // --- State Variables ---
  public enum HoodGoal {
    IDLE, // 待机/刹车
    TRACKING, // 视觉角度追踪
    FIXED_ANGLE, // 定点角度 (使用 fixedAngleRads)
    TEST, // 测试模式 (读取 kFixAngle)
    ZEROING, // 归零状态
    PASSING, // 传球角度
  }

  @Getter @Setter @AutoLogOutput private HoodGoal goal = HoodGoal.IDLE;

  // 供 FIXED_ANGLE 模式使用
  @Setter private double fixedAngleRads = HoodConstants.kHoodInitialAngle;

  @Getter @AutoLogOutput private boolean hoodZeroed = false;

  @Getter
  @Accessors(fluent = true)
  @AutoLogOutput
  private boolean atGoal = false;

  // --- Hardware Safety & Alerts ---
  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert motorDisconnectedAlert =
      new Alert("Hood motor disconnected!", Alert.AlertType.kWarning);

  public Hood(HoodIO io) {
    this.io = io;
    io.setPID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get(), kG.get());
  }

  @Override
  public void periodic() {
    // 1. 读取输入
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    // 2. 更新硬件报警与 Tunables
    motorDisconnectedAlert.set(!motorConnectedDebouncer.calculate(inputs.motorConnected));
    updateTunables();

    // 3. 核心控制循环 (状态机)
    // 如果系统被禁用或未归零，强制刹车
    if (DriverStation.isDisabled() || !hoodZeroed) {
      outputs.mode = HoodIOOutputMode.BRAKE;
      outputs.velocityRadsPerSec = 0.0;
      atGoal = false;
    } else {
      switch (goal) {
        case IDLE -> {
          // 这里可以设为 COAST 或 BRAKE，视你的物理结构是否需要保持力
          outputs.mode = HoodIOOutputMode.BRAKE;
          outputs.velocityRadsPerSec = 0.0;
          atGoal = true;
        }
        case TRACKING -> {
          var params = ShotCalculator.getInstance().getParameters();
          runPositionLogic(params.hoodAngle(), params.hoodVelocity());
        }
        case FIXED_ANGLE -> {
          runPositionLogic(HoodConstants.kFixAngle, HoodConstants.kFixVelocity);
        }
        case TEST -> {
          runPositionLogic(kFixAngle.get(), HoodConstants.kFixVelocity);
        }
        case ZEROING -> {
          runPositionLogic(HoodConstants.kHoodInitialAngle, HoodConstants.kFixVelocity);
        }
        case PASSING -> {
          runPositionLogic(HoodConstants.kHoodPassingAngle, HoodConstants.kFixVelocity);
        }
      }
    }
    // 4. 应用输出
    Logger.recordOutput("Hood/Profile/mode", outputs.mode);
    Logger.recordOutput("Hood/Profile/zero", hoodZeroed);
    io.applyOutputs(outputs);
  }

  /** 内部位置闭环辅助方法：负责 clamp 角度、设置 output 并计算 atGoal */
  private void runPositionLogic(double targetAngleRads, double targetVelocityRadsPerSec) {
    double clampedAngle =
        MathUtil.clamp(targetAngleRads, HoodConstants.kHoodMinAngle, HoodConstants.kHoodMaxAngle);

    outputs.mode = HoodIOOutputMode.CLOSED_LOOP;
    outputs.positionRad = clampedAngle;
    outputs.velocityRadsPerSec = targetVelocityRadsPerSec;

    // 计算是否到位
    atGoal = Math.abs(inputs.positionRads - clampedAngle) <= toleranceDeg.get();

    // Log 目标值
    Logger.recordOutput("Hood/Profile/GoalPositionRad", clampedAngle);
    Logger.recordOutput("Hood/Profile/GoalVelocityRadPerSec", targetVelocityRadsPerSec);
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
  public Command setGoalCommand(HoodGoal newGoal) {
    return Commands.runOnce(() -> this.goal = newGoal, this).ignoringDisable(true);
  }

  /** 归零命令 */
  public Command zeroCommand() {
    return runOnce(
            () -> {
              hoodZeroed = true;
              this.goal = HoodGoal.IDLE;
            })
        .ignoringDisable(true);
  }
}
