package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.hood.HoodIO.HoodIOOutputMode;
import frc.robot.subsystems.shooter.hood.HoodIO.HoodIOOutputs;
import frc.robot.util.FullSubsystem;
import frc.robot.util.Geoffrey.PhysicalJoint;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hood extends FullSubsystem {

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
  private final PhysicalJoint muzzleJoint;

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

  public Hood(HoodIO io, PhysicalJoint muzzleJoint) {
    this.io = io;
    this.muzzleJoint = muzzleJoint;
    io.setPID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get(), kG.get());
  }

  @Override
  public void updateInputsPeriodic() {
    // 1. 读取输入
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    // 2. 更新硬件报警与 Tunables
    motorDisconnectedAlert.set(!motorConnectedDebouncer.calculate(inputs.motorConnected));
    updateTunables();
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled() || !hoodZeroed) {
      outputs.mode = HoodIOOutputMode.COAST;
      outputs.velocityRadsPerSec = 0.0;
      atGoal = false;
    } else {
      switch (goal) {
        case IDLE -> {
          // 这里可以设为 COAST 或 BRAKE，视你的物理结构是否需要保持力
          outputs.mode = HoodIOOutputMode.COAST;
          outputs.velocityRadsPerSec = 0.0;
          atGoal = true;
        }
        case TRACKING -> {
          // Translation2d target =
          //     AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
          // ShooterSetpoint sp =
          //     ShooterSetpoint.makeSetpoint(
          //         muzzleJoint,
          //         target,
          //         FieldConstants.hMax,
          //         HoodConstants.kHoodMinAngle,
          //         HoodConstants.kHoodMaxAngle,
          //         TrajectoryConfig.getHubConfig());
          // runPositionFOCLogic(
          //     sp.hoodPositionRadians,
          //     sp.hoodVelocityRadsPerSec,
          //     sp.hoodAccelerationRadsPerSecSquared,
          //     0.0);
        }
        case PASSING -> {
          // Translation2d target = getBestPassingTarget();
          // ShooterSetpoint sp =
          //     ShooterSetpoint.makeSetpoint(
          //         muzzleJoint,
          //         target,
          //         FieldConstants.hMax,
          //         HoodConstants.kHoodMinAngle,
          //         HoodConstants.kHoodMaxAngle,
          //         TrajectoryConfig.getPassingConfig());
          // runPositionFOCLogic(
          //     sp.hoodPositionRadians,
          //     sp.hoodVelocityRadsPerSec,
          //     sp.hoodAccelerationRadsPerSecSquared,
          //     0.0);
        }
        case FIXED_ANGLE -> {
          runPositionFOCLogic((Math.PI / 2) - fixedAngleRads, 0.0, 0.0, 0.0);
        }
        case TEST -> {
          runPositionFOCLogic((Math.PI / 2) - kFixAngle.get(), 0.0, 0.0, 0.0);
        }
        case ZEROING -> {
          runPositionFOCLogic((Math.PI / 2) - HoodConstants.kHoodInitialAngle, 0.0, 0.0, 0.0);
        }
      }
    }
  }

  @Override
  public void executePeriodic() {
    // 4. 应用输出
    Logger.recordOutput("Hood/Profile/mode", outputs.mode);
    Logger.recordOutput("Hood/Profile/zero", hoodZeroed);
    io.applyOutputs(outputs);
  }

  public void runPositionFOCLogic(
      double targetAngleRads,
      double targetVelocityRadsPerSec,
      double targetAccelation,
      double feedforwardAmps) {
    double targetAngleRadsCoangle = (Math.PI / 2) - targetAngleRads;
    double clampedAngle =
        MathUtil.clamp(
            targetAngleRadsCoangle, HoodConstants.kHoodMinAngle, HoodConstants.kHoodMaxAngle);

    outputs.mode = HoodIOOutputMode.POSITION_FOC;
    outputs.positionRads = clampedAngle;
    outputs.velocityRadsPerSec = -targetVelocityRadsPerSec;
    outputs.accelerationRadPerSec2 = -targetAccelation;
    outputs.feedforwardAmps = feedforwardAmps;

    // 计算是否到位
    atGoal = Math.abs(inputs.positionRads - clampedAngle) <= toleranceDeg.get();

    // Log 目标值
    Logger.recordOutput("Hood/Profile/GoalPositionRad", clampedAngle);
    Logger.recordOutput("Hood/Profile/GoalVelocityRadPerSec", targetVelocityRadsPerSec);
    Logger.recordOutput("Hood/Profile/PositionRad", targetAngleRads);
    Logger.recordOutput("Hood/Profile/GoalAccelation", targetAccelation);
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
