package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.subsystems.flywheel.FlywheelIO.FlywheelIOOutputMode;
import frc.robot.subsystems.flywheel.FlywheelIO.FlywheelIOOutputs;
import frc.robot.util.FullSubsystem;
import frc.robot.util.Geoffrey.PhysicalJoint;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * The Flywheel acts as both the intake and the shooter. Spin it one way to suck a ball in, and spin
 * it fast to shoot. It is a single velocity-controlled motor.
 */
public class Flywheel extends FullSubsystem {
  // --- Tunables ---
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Flywheel/kP");
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Flywheel/kI");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Flywheel/kD");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Flywheel/kS");
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Flywheel/kV");
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Flywheel/kA");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Flywheel/kG");

  private static final LoggedTunableNumber kTestVelocity =
      new LoggedTunableNumber("Flywheel/kTestVelocity");
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
      kTestVelocity.initDefault(100.0);
    } else {
      kP.initDefault(FlywheelConstants.kP);
      kI.initDefault(FlywheelConstants.kI);
      kD.initDefault(FlywheelConstants.kD);
      kS.initDefault(FlywheelConstants.kS);
      kV.initDefault(FlywheelConstants.kV);
      kA.initDefault(FlywheelConstants.kA);
      kG.initDefault(FlywheelConstants.kG);
      velocityTolerance.initDefault(FlywheelConstants.kVelocityTolerance);
      kTestVelocity.initDefault(FlywheelConstants.kShootVelocity);
    }
  }

  // --- IO & Inputs ---
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private final FlywheelIOOutputs outputs = new FlywheelIOOutputs();

  /** The muzzle joint (on the arm) in the Geoffrey chain, kept for future shot calculation. */
  private final PhysicalJoint muzzleJoint;

  // --- State ---
  public enum FlywheelGoal {
    IDLE, // stopped
    INTAKE, // spin in to suck a ball in
    SHOOT, // spin fast to shoot (fixed velocity)
    TRACKING, // follow a dynamic velocity setpoint (e.g. trajectory-based shooting)
    OUTTAKE, // reverse to spit out
    TEST // read the tunable test velocity
  }

  @Getter @Setter @AutoLogOutput private FlywheelGoal goal = FlywheelGoal.IDLE;

  /** Dynamic velocity setpoint used by the TRACKING goal (rad/s). Set from aiming logic. */
  @Setter private double trackingVelocityRadsPerSec = 0.0;

  /** Dynamic acceleration feedforward used by the TRACKING goal (rad/s^2). */
  @Setter private double trackingAccelerationRadsPerSec2 = 0.0;

  @Getter
  @Accessors(fluent = true)
  @AutoLogOutput
  private boolean atGoal = false;

  // --- Alerts ---
  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Alert disconnected =
      new Alert("Flywheel motor disconnected!", Alert.AlertType.kWarning);
  private Debouncer atGoalDebouncer = new Debouncer(atGoalDebounce.get(), DebounceType.kFalling);

  public Flywheel(FlywheelIO io, PhysicalJoint muzzleJoint) {
    this.io = io;
    this.muzzleJoint = muzzleJoint;
    io.setPID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get(), kG.get());
  }

  @Override
  public void updateInputsPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
    updateTunables();
    disconnected.set(!motorConnectedDebouncer.calculate(inputs.connected));
  }

  @AutoLogOutput private double stuckStartTime = -1;

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      outputs.mode = FlywheelIOOutputMode.COAST;
      outputs.velocityRadsPerSec = 0.0;
      atGoal = false;
    } else {
      switch (goal) {
        case IDLE -> {
          outputs.mode = FlywheelIOOutputMode.COAST;
          outputs.velocityRadsPerSec = 0.0;
          atGoal = false;
        }
        case INTAKE -> {
          runVelocity(FlywheelConstants.kIntakeVelocity, 2);
          if (inputs.velocityRadsPerSec < FlywheelConstants.stuckThresholdVelocity) {
            if ((stuckStartTime < 0)) {
              stuckStartTime = Timer.getFPGATimestamp();
            }

          } else {
            stuckStartTime = -1;
          }
          if (stuckStartTime > 0) {
            if (Timer.getFPGATimestamp() - stuckStartTime
                >= FlywheelConstants.stuckThresholdTimeS) {
              runVelocity(-FlywheelConstants.kIntakeVelocity, 3);
            }
            if (Timer.getFPGATimestamp() - stuckStartTime
                >= FlywheelConstants.stuckThresholdTimeS + FlywheelConstants.backSpinTime) {
              stuckStartTime = -1;
            }
          }
        }
        case SHOOT -> runVelocity(FlywheelConstants.kShootVelocity, 3);
        case TRACKING -> runVelocity(
            trackingVelocityRadsPerSec, trackingAccelerationRadsPerSec2, 3);
        case OUTTAKE -> runVelocity(FlywheelConstants.kOuttakeVelocity, 3);
        case TEST -> runVelocity(kTestVelocity.get());
      }
    }
  }

  @Override
  public void executePeriodic() {
    Logger.recordOutput("Flywheel/Mode", outputs.mode);
    Logger.recordOutput("Flywheel/Setpoint", outputs.velocityRadsPerSec);
    io.applyOutputs(outputs);
    Robot.batteryLogger.reportCurrentUsage(
        "Flywheel", false, inputs.connected ? inputs.supplyCurrentAmps : 0.0);
  }

  public void runVelocity(double velocityRadsPerSec) {
    runVelocity(velocityRadsPerSec, 0.0, 0);
  }

  public void runVelocity(double velocityRadsPerSec, int num_motors) {
    runVelocity(velocityRadsPerSec, 0.0, num_motors);
  }

  public void runVelocity(
      double velocityRadsPerSec, double accelerationRadsPerSec2, int num_motors) {
    outputs.mode = FlywheelIOOutputMode.VELOCITY;
    outputs.velocityRadsPerSec = velocityRadsPerSec;
    outputs.accelerationRadPerSec2 = accelerationRadsPerSec2;
    outputs.volts = 0.0;
    outputs.num_motors = num_motors;

    boolean inTolerance =
        Math.abs(inputs.velocityRadsPerSec - velocityRadsPerSec) <= velocityTolerance.get();
    if (Math.abs(velocityRadsPerSec) < 1.0) {
      inTolerance = false;
    }
    atGoal = atGoalDebouncer.calculate(inTolerance);
  }

  public boolean isAtGoal() {
    return atGoal;
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
        || kA.hasChanged(hashCode())
        || kG.hasChanged(hashCode())) {
      io.setPID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get(), kG.get());
    }
  }

  public double getVelocity() {
    return inputs.velocityRadsPerSec;
  }

  // --- Commands ---
  public Command setGoalCommand(FlywheelGoal newGoal) {
    return Commands.runOnce(() -> this.goal = newGoal, this);
  }
}
