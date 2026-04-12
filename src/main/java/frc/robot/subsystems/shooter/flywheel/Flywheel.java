package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO.FlywheelIOOutputMode;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO.FlywheelIOOutputs;
import frc.robot.subsystems.shooter.hood.HoodConstants;
import frc.robot.util.FullSubsystem;
import frc.robot.util.Geoffrey.PhysicalJoint;
import frc.robot.util.Geoffrey.ShooterSetpoint;
import frc.robot.util.Geoffrey.TrajectoryCalculator;
import frc.robot.util.Geoffrey.TrajectoryConfig;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.geometry.AllianceFlipUtil;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends FullSubsystem {
  // --- Tunable Numbers ---
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Flywheel/kP");
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Flywheel/kI");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Flywheel/kD");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Flywheel/kS");
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Flywheel/kV");
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Flywheel/kA");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Flywheel/kG");

  private static final LoggedTunableNumber kFixVelocity =
      new LoggedTunableNumber("Flywheel/kFixVelocity");
  private static final LoggedTunableNumber velocityTolerance =
      new LoggedTunableNumber("Flywheel/VelocityTolerance"); // rad/s
  private static final LoggedTunableNumber atGoalDebounce =
      new LoggedTunableNumber("Flywheel/AtGoalDebounce", 0.2);

  static {
    if (Robot.isSimulation()) {
      kP.initDefault(0.035);
      kI.initDefault(0.0);
      kD.initDefault(0.1);
      kS.initDefault(0.0);
      kV.initDefault(0.0);
      kA.initDefault(0.0);
      kG.initDefault(0.0);
      velocityTolerance.initDefault(1.0);
      kFixVelocity.initDefault(100.0);
    } else {
      kP.initDefault(FlywheelConstants.kP);
      kI.initDefault(FlywheelConstants.kI);
      kD.initDefault(FlywheelConstants.kD);
      kS.initDefault(FlywheelConstants.kS);
      kV.initDefault(FlywheelConstants.kV);
      kA.initDefault(FlywheelConstants.kA);
      kG.initDefault(FlywheelConstants.kG);
      velocityTolerance.initDefault(FlywheelConstants.kVelocityTolerance);
      kFixVelocity.initDefault(FlywheelConstants.kFixVelocity);
    }
  }

  // --- IO & Inputs ---
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private final FlywheelIOOutputs outputs = new FlywheelIOOutputs();
  private final PhysicalJoint muzzleJoint;

  // --- State Variables ---
  public enum FlywheelGoal {
    IDLE, // 停止/空转
    TRACKING, // 视觉解算速度
    FIXED_VELOCITY, // 定点固定速度 (使用 fixedVelocity 参数)
    TEST, // 测试模式 (读取 TunableNumber)
    PASSING,
    OUTTAKE,
    ACTIVE,
  }

  @Getter @Setter @AutoLogOutput private FlywheelGoal goal = FlywheelGoal.IDLE;

  @Setter private double fixedVelocity = FlywheelConstants.kFixVelocity;

  @Getter
  @Accessors(fluent = true)
  @AutoLogOutput
  private boolean atGoal = false;

  @AutoLogOutput private long shotCount = 0;

  // --- Hardware Safety & Alerts ---
  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Alert disconnected;
  private Debouncer atGoalDebouncer = new Debouncer(atGoalDebounce.get(), DebounceType.kFalling);

  public Flywheel(FlywheelIO io, PhysicalJoint muzzleJoint) {
    this.io = io;
    this.muzzleJoint = muzzleJoint;
    disconnected = new Alert("Flywheel motor disconnected!", Alert.AlertType.kWarning);

    io.setPID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get(), kG.get());
  }

  @Override
  public void updateInputsPeriodic() {
    // 1. 读取输入
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    // 2. 更新 Tunables 和 硬件报警
    updateTunables();
    disconnected.set(!motorConnectedDebouncer.calculate(inputs.connected));
  }

  @Override
  public void periodic() {
    // 3. 核心状态机逻辑
    if (DriverStation.isDisabled()) {
      outputs.mode = FlywheelIOOutputMode.COAST;
      outputs.velocityRadsPerSec = 0.0;
      outputs.volts = 0.0;
      atGoal = false;
    } else {
      switch (goal) {
        case IDLE -> {
          outputs.mode = FlywheelIOOutputMode.COAST;
          outputs.velocityRadsPerSec = 0.0;
          outputs.volts = 0.0;
          atGoal = false;
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
          // double radPerSec =
          //     TrajectoryCalculator.getFlywheelSetpoint(sp.shooterVelocityMetersPerSec)
          //         / FlywheelConstants.kFlywheelRadius;
          // double radPerSec2 =
          //     TrajectoryCalculator.getFlywheelAcceleration(
          //             sp.shooterVelocityMetersPerSec, sp.shooterAccelerationMetersPerSecSquared)
          //         / FlywheelConstants.kFlywheelRadius;
          // runVelocityFOCLogic(radPerSec, radPerSec2, 0.0);
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
          // double radPerSec =
          //     TrajectoryCalculator.getFlywheelSetpoint(sp.shooterVelocityMetersPerSec)
          //         / FlywheelConstants.kFlywheelRadius;
          // double radPerSec2 =
          //     TrajectoryCalculator.getFlywheelAcceleration(
          //             sp.shooterVelocityMetersPerSec, sp.shooterAccelerationMetersPerSecSquared)
          //         / FlywheelConstants.kFlywheelRadius;
          // runVelocityFOCLogic(radPerSec, radPerSec2, 0.0);
        }
        case ACTIVE -> {
          Translation2d target =
              AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
          ShooterSetpoint sp =
              ShooterSetpoint.makeSetpoint(
                  muzzleJoint,
                  target,
                  FieldConstants.hMax,
                  HoodConstants.kHoodMinAngle,
                  HoodConstants.kHoodMaxAngle,
                  TrajectoryConfig.getHubConfig());
          double radPerSec =
              (TrajectoryCalculator.getFlywheelSetpoint(sp.shooterVelocityMetersPerSec)
                      / FlywheelConstants.kFlywheelRadius)
                  * FlywheelConstants.kActiveRatio;
          double radPerSec2 =
              TrajectoryCalculator.getFlywheelAcceleration(
                      sp.shooterVelocityMetersPerSec, sp.shooterAccelerationMetersPerSecSquared)
                  / FlywheelConstants.kFlywheelRadius;
          runVelocityFOCLogic(radPerSec, radPerSec2, 0.0);
        }
        case FIXED_VELOCITY -> {
          runVelocityFOCLogic(FlywheelConstants.kFixVelocity, 0.0, 0.0);
        }
        case TEST -> {
          runVelocityFOCLogic(kFixVelocity.get(), 0.0, 0.0);
        }
        case OUTTAKE -> {
          runVelocityFOCLogic(FlywheelConstants.kOutTakeVelocity, 0.0, 0.0);
        }
      }
    }
  }

  @Override
  public void executePeriodic() {
    Logger.recordOutput("Flywheel/Mode", outputs.mode);
    Logger.recordOutput("Flywheel/Setpoint", outputs.velocityRadsPerSec);
    Logger.recordOutput("FlyWheel/GoalAcceleration", outputs.accelerationRadPerSec2);
    io.applyOutputs(outputs);
  }

  public void runVelocityFOCLogic(
      double velocityRadsPerSec, double acelerationRadPerSec2, double feedforwardAmps) {
    outputs.mode = FlywheelIOOutputMode.VELOCITY_FOC;
    outputs.velocityRadsPerSec = velocityRadsPerSec;
    outputs.accelerationRadPerSec2 = acelerationRadPerSec2;
    outputs.feedforwardAmps = feedforwardAmps; // 这里直接用电压作为前馈，具体实现时可能需要转换为电流

    outputs.volts = 0.0; // 清零电压，防止干扰闭环

    // 计算是否到达目标
    boolean inTolerance =
        Math.abs(inputs.velocityRadsPerSec - velocityRadsPerSec) <= velocityTolerance.get();

    // 如果设定值过低，强制认为未就绪
    if (Math.abs(velocityRadsPerSec) < 1.0) {
      inTolerance = false;
    }

    atGoal = atGoalDebouncer.calculate(inTolerance);
  }


  /** 更新可调参数 */
  private void updateTunables() {
    if (atGoalDebounce.hasChanged(hashCode())) {
      atGoalDebouncer = new Debouncer(atGoalDebounce.get(), DebounceType.kFalling);
    }
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

  public double getVelocity() {
    return inputs.velocityRadsPerSec;
  }

  // --- Commands (供 Superstructure 调用) ---
  public Command setGoalCommand(FlywheelGoal newGoal) {
    return Commands.runOnce(() -> this.goal = newGoal, this);
  }
}
