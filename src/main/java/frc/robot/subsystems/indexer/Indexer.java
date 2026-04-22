package frc.robot.subsystems.indexer;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.subsystems.indexer.IndexerIO.IndexerIOOutputMode;
import frc.robot.subsystems.indexer.IndexerIO.IndexerIOOutputs;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Indexer extends FullSubsystem {
  // --- Indexer Tunable Numbers ---
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Indexer/kP");
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Indexer/kI");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Indexer/kD");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Indexer/kS");
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Indexer/kV");
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Indexer/kA");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Indexer/kG");

  private static final LoggedTunableNumber kIntakeVelocity =
      new LoggedTunableNumber("Indexer/kIntakeVelocity");
  private static final LoggedTunableNumber kOuttakeVelocity =
      new LoggedTunableNumber("Indexer/kOuttakeVelocity");
  private static final LoggedTunableNumber kActiveVelocity =
      new LoggedTunableNumber("Indexer/kActiveVelocity");
  private static final LoggedTunableNumber kShootVelocity =
      new LoggedTunableNumber("Indexer/kShootVelocity");
  private static final LoggedTunableNumber velocityTolerance =
      new LoggedTunableNumber("Indexer/VelocityTolerance"); // rad/s
  private static final LoggedTunableNumber atGoalDebounce =
      new LoggedTunableNumber("Indexer/AtGoalDebounce", 0.2);

  // --- Triggers Tunable Numbers ---
  private static final LoggedTunableNumber kTriggers_kP =
      new LoggedTunableNumber("Indexer/Triggers_kP");
  private static final LoggedTunableNumber kTriggers_kI =
      new LoggedTunableNumber("Indexer/Triggers_kI");
  private static final LoggedTunableNumber kTriggers_kD =
      new LoggedTunableNumber("Indexer/Triggers_kD");
  private static final LoggedTunableNumber kTriggers_kS =
      new LoggedTunableNumber("Indexer/Triggers_kS");
  private static final LoggedTunableNumber kTriggers_kV =
      new LoggedTunableNumber("Indexer/Triggers_kV");
  private static final LoggedTunableNumber kTriggers_kA =
      new LoggedTunableNumber("Indexer/Triggers_kA");
  private static final LoggedTunableNumber kTriggers_kG =
      new LoggedTunableNumber("Indexer/Triggers_kG");

  private static final LoggedTunableNumber kTriggersIntakeVelocity =
      new LoggedTunableNumber("Indexer/TriggersIntakeVelocity");
  private static final LoggedTunableNumber kTriggersOuttakeVelocity =
      new LoggedTunableNumber("Indexer/TriggersOuttakeVelocity");
  private static final LoggedTunableNumber kTriggersShootVelocity =
      new LoggedTunableNumber("Indexer/TriggersShootVelocity");

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
      kActiveVelocity.initDefault(100.0);
      kShootVelocity.initDefault(100.0);

      kTriggers_kP.initDefault(0.035);
      kTriggers_kI.initDefault(0.0);
      kTriggers_kD.initDefault(0.0);
      kTriggers_kS.initDefault(0.0);
      kTriggers_kV.initDefault(0.0);
      kTriggers_kA.initDefault(0.0);
      kTriggers_kG.initDefault(0.0);
      kTriggersIntakeVelocity.initDefault(150.0);
      kTriggersOuttakeVelocity.initDefault(-150.0);
      kTriggersShootVelocity.initDefault(150.0);
    } else {
      kP.initDefault(IndexerConstants.kP);
      kI.initDefault(IndexerConstants.kI);
      kD.initDefault(IndexerConstants.kD);
      kS.initDefault(IndexerConstants.kS);
      kV.initDefault(IndexerConstants.kV);
      kA.initDefault(IndexerConstants.kA);
      kG.initDefault(IndexerConstants.kG);
      velocityTolerance.initDefault(IndexerConstants.kVelocityTolerance);
      kIntakeVelocity.initDefault(IndexerConstants.kIntakeVelocity);
      kOuttakeVelocity.initDefault(IndexerConstants.kOuttakeVelocity);
      kActiveVelocity.initDefault(IndexerConstants.kActiveVelocity);
      kShootVelocity.initDefault(IndexerConstants.kShootVelocity);

      kTriggers_kP.initDefault(IndexerConstants.kTriggers_kP);
      kTriggers_kI.initDefault(0.0);
      kTriggers_kD.initDefault(0.0);
      kTriggers_kS.initDefault(0.0);
      kTriggers_kV.initDefault(0.0);
      kTriggers_kA.initDefault(0.0);
      kTriggers_kG.initDefault(0.0);
      kTriggersIntakeVelocity.initDefault(IndexerConstants.kTriggersIntakeVelocity);
      kTriggersOuttakeVelocity.initDefault(IndexerConstants.kTriggersOuttakeVelocity);
      kTriggersShootVelocity.initDefault(IndexerConstants.kTriggersShootVelocity);
    }
  }

  // --- IO & Inputs ---
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
  private final IndexerIOOutputs outputs = new IndexerIOOutputs();

  public enum IndexerGoal {
    INTAKE,
    SHOOT,
    OUTTAKE,
    ACTIVE,
    STOP
  }

  @Getter @Setter @AutoLogOutput private IndexerGoal goal = IndexerGoal.STOP;

  @Getter
  @Accessors(fluent = true)
  @AutoLogOutput
  private boolean atGoal = false;

  // --- Hardware Safety & Alerts ---
  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Debouncer rightmotorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Debouncer triggersConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Debouncer righttriggersConnectedDebouncer =
      new Debouncer(0.5, DebounceType.kFalling);
  private final Debouncer leftLimitSwitchconnectedDebouncer =
      new Debouncer(0.5, DebounceType.kFalling);
  private final Debouncer rightLimitSwitchconnectedDebouncer =
      new Debouncer(0.5, DebounceType.kFalling);
  private final Alert rightdisconnected;
  private final Alert disconnected;
  private final Alert triggersDisconnected;
  private final Alert righttriggersDisconnected;
  private final Alert leftLimitSwitchconnected;
  private final Alert rightLimitSwitchconnected;

  private Debouncer atGoalDebouncer = new Debouncer(atGoalDebounce.get(), DebounceType.kFalling);

  public Indexer(IndexerIO io) {
    this.io = io;
    disconnected = new Alert("Indexer motor disconnected!", Alert.AlertType.kWarning);
    rightdisconnected = new Alert("RightIndexer motor disconnected!", Alert.AlertType.kWarning);
    triggersDisconnected = new Alert("Triggers motors disconnected!", Alert.AlertType.kWarning);
    righttriggersDisconnected =
        new Alert("RightTriggers motors disconnected!", Alert.AlertType.kWarning);
    leftLimitSwitchconnected = new Alert("leftLimitSwitch disconnected!", Alert.AlertType.kWarning);
    rightLimitSwitchconnected =
        new Alert("rightLimitSwitch disconnected!", Alert.AlertType.kWarning);

    io.setPID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get(), kG.get());
    io.setTriggersPID(
        kTriggers_kP.get(),
        kTriggers_kI.get(),
        kTriggers_kD.get(),
        kTriggers_kS.get(),
        kTriggers_kV.get(),
        kTriggers_kA.get(),
        kTriggers_kG.get());
  }

  @Override
  public void updateInputsPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);

    updateTunables();
    disconnected.set(!motorConnectedDebouncer.calculate(inputs.connected));
    rightdisconnected.set(!rightmotorConnectedDebouncer.calculate(inputs.indexRightconnected));
    triggersDisconnected.set(!triggersConnectedDebouncer.calculate(inputs.triggersConnected));
    righttriggersDisconnected.set(
        !righttriggersConnectedDebouncer.calculate(inputs.triggersRightconnected));
    leftLimitSwitchconnected.set(
        !triggersConnectedDebouncer.calculate(inputs.leftLimitSwitchconnected));
    rightLimitSwitchconnected.set(
        !righttriggersConnectedDebouncer.calculate(inputs.rightLimitSwitchconnected));
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      outputs.mode = IndexerIOOutputMode.COAST;
      outputs.velocityRadsPerSec = 0.0;
      outputs.triggersMode = IndexerIOOutputMode.COAST;
      outputs.triggersVelocityRadsPerSec = 0.0;
      atGoal = false;
    } else {
      switch (goal) {
        case STOP -> {
          outputs.mode = IndexerIOOutputMode.COAST;
          outputs.velocityRadsPerSec = 0.0;
          // Triggers also stop
          outputs.triggersMode = IndexerIOOutputMode.COAST;
          outputs.triggersVelocityRadsPerSec = 0.0;
          atGoal = false;
        }
        case INTAKE -> {
          // Indexer runs intake, triggers stop (game piece fed by indexer only)
          runVelocityLogic(kIntakeVelocity.get());
          outputs.triggersMode = IndexerIOOutputMode.COAST;
          outputs.triggersVelocityRadsPerSec = 0.0;
        }
        case ACTIVE -> {
          runVelocityLogic(kActiveVelocity.get());
          // Triggers stop in active pre-aim mode
          outputs.triggersMode = IndexerIOOutputMode.COAST;
          outputs.triggersVelocityRadsPerSec = 0.0;
        }
        case OUTTAKE -> {
          runVelocityLogic(kOuttakeVelocity.get());
          // Triggers also outtake
          outputs.triggersMode = IndexerIOOutputMode.VELOCITY;
          outputs.triggersVelocityRadsPerSec = kTriggersOuttakeVelocity.get();
        }
        case SHOOT -> {
          runVelocityLogic(kShootVelocity.get());

          outputs.mode = IndexerIOOutputMode.FEEDING;
          outputs.maxVelocityRadsPerSec = kShootVelocity.get();
          outputs.maxCurrent = 80;
          outputs.velocityRadsPerSec = kIntakeVelocity.get();
          atGoal = true;
          // Triggers also shoot
          outputs.triggersMode = IndexerIOOutputMode.FEEDING;
          outputs.triggersVelocityRadsPerSec = kTriggersShootVelocity.get();
        }
      }
    }
  }

  @Override
  public void executePeriodic() {
    Logger.recordOutput("Indexer/Mode", outputs.mode);
    Logger.recordOutput("Indexer/Setpoint", outputs.velocityRadsPerSec);
    Logger.recordOutput("Indexer/TriggersMode", outputs.triggersMode);
    Logger.recordOutput("Indexer/TriggersSetpoint", outputs.triggersVelocityRadsPerSec);
    Logger.recordOutput("Indexer/FeedingLeft", inputs.feedingLeft);
    Logger.recordOutput("Indexer/LeftLimitSwitch", inputs.leftLimitSwitch);
    Logger.recordOutput("Indexer/RightLimitSwitch", inputs.rightLimitSwitch);
    io.applyOutputs(outputs);
    Robot.batteryLogger.reportCurrentUsage(
        "Indexer", false, inputs.connected ? inputs.supplyCurrentAmps : 0.0);
  }

  /** Internal velocity closed-loop helper: sets output and computes atGoal */
  private void runVelocityLogic(double velocityRadsPerSec) {
    outputs.mode = IndexerIOOutputMode.VELOCITY;
    outputs.velocityRadsPerSec = velocityRadsPerSec;

    boolean inTolerance =
        Math.abs(inputs.velocityRadsPerSec - velocityRadsPerSec) <= velocityTolerance.get();

    if (Math.abs(velocityRadsPerSec) < 1.0) {
      inTolerance = false;
    }

    atGoal = atGoalDebouncer.calculate(inTolerance);
  }

  /** Update tunable parameters */
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
    if (kTriggers_kP.hasChanged(hashCode())
        || kTriggers_kI.hasChanged(hashCode())
        || kTriggers_kD.hasChanged(hashCode())
        || kTriggers_kS.hasChanged(hashCode())
        || kTriggers_kV.hasChanged(hashCode())
        || kTriggers_kA.hasChanged(hashCode())
        || kTriggers_kG.hasChanged(hashCode())) {
      io.setTriggersPID(
          kTriggers_kP.get(),
          kTriggers_kI.get(),
          kTriggers_kD.get(),
          kTriggers_kS.get(),
          kTriggers_kV.get(),
          kTriggers_kA.get(),
          kTriggers_kG.get());
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
