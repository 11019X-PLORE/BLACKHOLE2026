package frc.robot.subsystems.indexer;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.indexer.IndexerIO.IndexerIOOutputMode;
import frc.robot.subsystems.indexer.IndexerIO.IndexerIOOutputs;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** A single roller motor that indexes the ball between the intake and the indexer. */
public class Indexer extends FullSubsystem {
  // --- Tunables ---
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Indexer/kP");
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Indexer/kI");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Indexer/kD");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Indexer/kS");
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Indexer/kV");
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Indexer/kA");

  private static final LoggedTunableNumber Trigger_kP = new LoggedTunableNumber("Trigger/kP");
  private static final LoggedTunableNumber Trigger_kI = new LoggedTunableNumber("Trigger/kI");
  private static final LoggedTunableNumber Trigger_kD = new LoggedTunableNumber("Trigger/kD");
  private static final LoggedTunableNumber Trigger_kS = new LoggedTunableNumber("Trigger/kS");
  private static final LoggedTunableNumber Trigger_kV = new LoggedTunableNumber("Trigger/kV");
  private static final LoggedTunableNumber Trigger_kA = new LoggedTunableNumber("Trigger/kA");

  private static final LoggedTunableNumber kIntakeVelocity =
      new LoggedTunableNumber("Indexer/kIntakeVelocity");
  private static final LoggedTunableNumber kIntakeTriggerVelocity =
      new LoggedTunableNumber("Indexer/kIntakeTriggerVelocity");
  private static final LoggedTunableNumber kShootVelocity =
      new LoggedTunableNumber("Indexer/kShootVelocity");
  private static final LoggedTunableNumber kShootTriggerVelocity =
      new LoggedTunableNumber("Indexer/kShootTriggerVelocity");
  private static final LoggedTunableNumber kOuttakeVelocity =
      new LoggedTunableNumber("Indexer/kOuttakeVelocity");
  private static final LoggedTunableNumber kOuttakeTriggerVelocity =
      new LoggedTunableNumber("Indexer/kOuttakeTriggerVelocity");
  private static final LoggedTunableNumber velocityTolerance =
      new LoggedTunableNumber("Indexer/VelocityTolerance"); // rad/s
  private static final LoggedTunableNumber atGoalDebounce =
      new LoggedTunableNumber("Indexer/AtGoalDebounce", 0.2);

  static {
    if (Robot.isSimulation()) {
      kP.initDefault(0.035);
      kI.initDefault(0.0);
      kD.initDefault(0.1);
      kS.initDefault(0.0);
      kV.initDefault(0.0);
      kA.initDefault(0.0);

      Trigger_kP.initDefault(0.035);
      Trigger_kI.initDefault(0.0);
      Trigger_kD.initDefault(0.1);
      Trigger_kS.initDefault(0.0);
      Trigger_kV.initDefault(0.0);
      Trigger_kA.initDefault(0.0);
      velocityTolerance.initDefault(10.0);
      kIntakeVelocity.initDefault(100.0);
      kIntakeTriggerVelocity.initDefault(100.0);
      kShootVelocity.initDefault(100.0);
      kShootTriggerVelocity.initDefault(100.0);
      kOuttakeVelocity.initDefault(-100.0);
      kOuttakeTriggerVelocity.initDefault(-100.0);
    } else {
      kP.initDefault(IndexerConstants.kP);
      kI.initDefault(IndexerConstants.kI);
      kD.initDefault(IndexerConstants.kD);
      kS.initDefault(IndexerConstants.kS);
      kV.initDefault(IndexerConstants.kV);
      kA.initDefault(IndexerConstants.kA);
      Trigger_kP.initDefault(IndexerConstants.triggerKP);
      Trigger_kI.initDefault(IndexerConstants.triggerKI);
      Trigger_kD.initDefault(IndexerConstants.triggerKD);
      Trigger_kS.initDefault(IndexerConstants.triggerKS);
      Trigger_kV.initDefault(IndexerConstants.triggerKV);
      Trigger_kA.initDefault(IndexerConstants.triggerKA);
      velocityTolerance.initDefault(IndexerConstants.kVelocityTolerance);
      kIntakeVelocity.initDefault(IndexerConstants.kIntakeVelocity);
      kIntakeTriggerVelocity.initDefault(IndexerConstants.kIntakeTriggerVelocity);
      kShootVelocity.initDefault(IndexerConstants.kShootVelocity);
      kShootTriggerVelocity.initDefault(IndexerConstants.kShootTriggerVelocity);
      kOuttakeVelocity.initDefault(IndexerConstants.kOuttakeVelocity);
      kOuttakeTriggerVelocity.initDefault(IndexerConstants.kOuttakeTriggerVelocity);
    }
  }

  // --- IO & Inputs ---
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
  private final IndexerIOOutputs outputs = new IndexerIOOutputs();

  // --- State ---
  public enum IndexerGoal {
    STOP,
    INTAKE, // index the ball inward
    SHOOT, // feed the ball to the indexer
    OUTTAKE, // reverse the ball out
    SLOWFORWARD
  }

  @Getter @Setter @AutoLogOutput private IndexerGoal goal = IndexerGoal.STOP;

  @Getter
  @Accessors(fluent = true)
  @AutoLogOutput
  private boolean atGoal = false;

  // --- Alerts ---
  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Alert disconnected =
      new Alert("Indexer motor disconnected!", Alert.AlertType.kWarning);
  private Debouncer atGoalDebouncer = new Debouncer(atGoalDebounce.get(), DebounceType.kFalling);
  private Arm arm;

  public Indexer(IndexerIO io, Arm arm) {
    this.io = io;
    io.setIndexerPID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get(), 0);
    io.setTriggerPID(
        Trigger_kP.get(),
        Trigger_kI.get(),
        Trigger_kD.get(),
        Trigger_kS.get(),
        Trigger_kV.get(),
        Trigger_kA.get(),
        0);
    this.arm = arm;
  }

  @Override
  public void updateInputsPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
    updateTunables();
    disconnected.set(!motorConnectedDebouncer.calculate(inputs.connected));
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      outputs.mode = IndexerIOOutputMode.COAST;
      outputs.indexerVelocityRadsPerSec = 0.0;
      outputs.triggerVelocityRadsPerSec = 0.0;
      atGoal = false;
    } else {
      switch (goal) {
        case STOP -> {
          if (arm.error >= Math.toRadians(5)) {
            runVelocity(50);
            outputs.triggerVelocityRadsPerSec = kIntakeTriggerVelocity.get();
          } else {
            outputs.mode = IndexerIOOutputMode.COAST;
            outputs.indexerVelocityRadsPerSec = 0.0;
            outputs.triggerVelocityRadsPerSec = 0.0;
            atGoal = false;
          }
        }
        case INTAKE -> {
          runVelocity(kIntakeVelocity.get());
          outputs.triggerVelocityRadsPerSec = kIntakeTriggerVelocity.get();
        }
        case SHOOT -> {
          runVelocity(kShootVelocity.get());

          outputs.triggerVelocityRadsPerSec = kShootTriggerVelocity.get();
        }
        case OUTTAKE -> {
          runVelocity(kOuttakeVelocity.get());
          outputs.triggerVelocityRadsPerSec = kOuttakeTriggerVelocity.get();
        }
        case SLOWFORWARD -> {
          runVelocity(50);
          outputs.triggerVelocityRadsPerSec = kIntakeTriggerVelocity.get();
        }
      }
    }
  }

  @Override
  public void executePeriodic() {
    Logger.recordOutput("Indexer/Mode", outputs.mode);
    Logger.recordOutput("Indexer/indexerSetpoint", outputs.indexerVelocityRadsPerSec);
    Logger.recordOutput("Indexer/TriggerSetpoint", outputs.triggerVelocityRadsPerSec);
    io.applyOutputs(outputs);
    Robot.batteryLogger.reportCurrentUsage(
        "Indexer", false, inputs.connected ? inputs.supplyCurrentAmps : 0.0);
  }

  public void runVelocity(double velocityRadsPerSec) {
    outputs.mode = IndexerIOOutputMode.VELOCITY;
    outputs.indexerVelocityRadsPerSec = velocityRadsPerSec;

    boolean inTolerance =
        Math.abs(inputs.velocityRadsPerSec - velocityRadsPerSec) <= velocityTolerance.get();
    if (Math.abs(velocityRadsPerSec) < 1.0) {
      inTolerance = false;
    }
    atGoal = atGoalDebouncer.calculate(inTolerance);
  }

  private void updateTunables() {
    if (atGoalDebounce.hasChanged(hashCode())) {
      atGoalDebouncer = new Debouncer(atGoalDebounce.get(), DebounceType.kFalling);
    }
    if (kP.hasChanged(hashCode())
        || kI.hasChanged(hashCode())
        || kD.hasChanged(hashCode())
        || kS.hasChanged(hashCode())
        || kV.hasChanged(hashCode())
        || kA.hasChanged(hashCode())) {
      io.setIndexerPID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get(), 0);
    }

    if (Trigger_kP.hasChanged(hashCode())
        || Trigger_kI.hasChanged(hashCode())
        || Trigger_kD.hasChanged(hashCode())
        || Trigger_kS.hasChanged(hashCode())
        || Trigger_kV.hasChanged(hashCode())
        || Trigger_kA.hasChanged(hashCode())) {
      io.setTriggerPID(
          Trigger_kP.get(),
          Trigger_kI.get(),
          Trigger_kD.get(),
          Trigger_kS.get(),
          Trigger_kV.get(),
          Trigger_kA.get(),
          0);
    }
  }

  public double getVelocity() {
    return inputs.velocityRadsPerSec;
  }

  // --- Commands ---
  public Command setGoalCommand(IndexerGoal newGoal) {
    return Commands.runOnce(() -> this.goal = newGoal, this);
  }
}
