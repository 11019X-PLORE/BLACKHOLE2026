package frc.robot.subsystems.triggers;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.subsystems.triggers.TriggersIO.TriggersIOOutputMode;
import frc.robot.subsystems.triggers.TriggersIO.TriggersIOOutputs;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Triggers extends FullSubsystem {
  // --- Tunable Numbers ---
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Triggers/kP");
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Triggers/kI");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Triggers/kD");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Triggers/kS");
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Triggers/kV");
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Triggers/kA");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Triggers/kG");

  private static final LoggedTunableNumber kIntakeVelocity =
      new LoggedTunableNumber("Triggers/kIntakeVelocity");
  private static final LoggedTunableNumber kOuttakeVelocity =
      new LoggedTunableNumber("Triggers/kOuttakeVelocity");
  private static final LoggedTunableNumber kShootVelocity =
      new LoggedTunableNumber("Triggers/kShootVelocity");
  private static final LoggedTunableNumber velocityTolerance =
      new LoggedTunableNumber("Triggers/VelocityTolerance"); // rad/s
  private static final LoggedTunableNumber atGoalDebounce =
      new LoggedTunableNumber("Triggers/AtGoalDebounce", 0.2);

  static {
    if (Robot.isSimulation()) {
      kP.initDefault(0.035);
      kI.initDefault(0.0);
      kD.initDefault(0.1);
      kS.initDefault(0.0);
      kV.initDefault(0.0);
      kA.initDefault(0.0);
      kG.initDefault(0.0);
      velocityTolerance.initDefault(10.0);
      kIntakeVelocity.initDefault(100.0);
      kOuttakeVelocity.initDefault(-100.0);
      kShootVelocity.initDefault(100.0);
    } else {
      kP.initDefault(TriggersConstants.kP);
      kI.initDefault(TriggersConstants.kI);
      kD.initDefault(TriggersConstants.kD);
      kS.initDefault(TriggersConstants.kS);
      kV.initDefault(TriggersConstants.kV);
      kA.initDefault(TriggersConstants.kA);
      kG.initDefault(TriggersConstants.kG);
      velocityTolerance.initDefault(TriggersConstants.kVelocityTolerance);
      kIntakeVelocity.initDefault(TriggersConstants.kIntakeVelocity);
      kOuttakeVelocity.initDefault(TriggersConstants.kOuttakeVelocity);
      kShootVelocity.initDefault(TriggersConstants.kShootVelocity);
    }
  }

  // --- IO & Inputs ---
  private final TriggersIO io;
  private final TriggersIOInputsAutoLogged inputs = new TriggersIOInputsAutoLogged();
  private final TriggersIOOutputs outputs = new TriggersIOOutputs();

  public enum TriggersGoal {
    INTAKE,
    SHOOT,
    OUTTAKE,
    STOP
  }

  @Getter @Setter @AutoLogOutput private TriggersGoal goal = TriggersGoal.STOP;

  @Getter
  @Accessors(fluent = true)
  @AutoLogOutput
  private boolean atGoal = false;

  // --- Hardware Safety & Alerts ---
  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Alert disconnected;
  private Debouncer atGoalDebouncer = new Debouncer(atGoalDebounce.get(), DebounceType.kFalling);

  public Triggers(TriggersIO io) {
    this.io = io;
    disconnected = new Alert("Triggers motor disconnected!", Alert.AlertType.kWarning);

    io.setPID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get(), kG.get());
  }

  @Override
  public void periodic() {
    // 1. 读取输入
    io.updateInputs(inputs);
    Logger.processInputs("Triggers", inputs);

    // 2. 更新 Tunables 和 硬件报警
    updateTunables();
    disconnected.set(!motorConnectedDebouncer.calculate(inputs.connected));
  }

  @Override
  public void periodicAfterScheduler() {
    if (DriverStation.isDisabled()) {
      outputs.mode = TriggersIOOutputMode.COAST;
      outputs.velocityRadsPerSec = 0.0;
      outputs.volts = 0.0;
      atGoal = false;
    } else {
      switch (goal) {
        case STOP -> {
          outputs.mode = TriggersIOOutputMode.COAST;
          outputs.velocityRadsPerSec = 0.0;
          outputs.volts = 0.0;
          atGoal = false;
        }
        case INTAKE -> {
          runVelocityLogic(kIntakeVelocity.get());
        }
        case OUTTAKE -> {
          runVelocityLogic(kOuttakeVelocity.get());
        }
        case SHOOT -> {
          runVelocityLogic(kShootVelocity.get());
        }
      }
    }
    Logger.recordOutput("Triggers/Mode", outputs.mode);
    Logger.recordOutput("Triggers/Setpoint", outputs.velocityRadsPerSec);
    io.applyOutputs(outputs);
  }

  /** 内部速度闭环辅助方法：负责设定 output 并计算 atGoal */
  private void runVelocityLogic(double velocityRadsPerSec) {
    outputs.mode = TriggersIOOutputMode.VELOCITY;
    outputs.velocityRadsPerSec = velocityRadsPerSec;
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
  public Command setGoalCommand(TriggersGoal newGoal) {
    return Commands.runOnce(() -> this.goal = newGoal, this);
  }
}
