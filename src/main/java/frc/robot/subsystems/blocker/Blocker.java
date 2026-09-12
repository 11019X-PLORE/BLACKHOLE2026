package frc.robot.subsystems.blocker;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.blocker.BlockerIO.BlockerIOOutputMode;
import frc.robot.subsystems.blocker.BlockerIO.BlockerIOOutputs;
import frc.robot.util.FullSubsystem;
import frc.robot.util.Geoffrey.PhysicalJoint;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;
import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * The Blocker is a single motor, angle-controlled mechanism that deploys to block the ball path. It
 * is part of the Geoffrey {@link PhysicalJoint} chain so its position can be located on the field.
 */
public class Blocker extends FullSubsystem implements PhysicalJoint {

  // --- Tunables ---
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Blocker/kP");
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Blocker/kI");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Blocker/kD");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Blocker/kS");
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Blocker/kV");
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Blocker/kA");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Blocker/kG");
  private static final LoggedTunableNumber kTestAngle =
      new LoggedTunableNumber("Blocker/kTestAngle");
  private static final LoggedTunableNumber toleranceRads =
      new LoggedTunableNumber("Blocker/ToleranceRads");

  static {
    if (Robot.isSimulation()) {
      kP.initDefault(5.0);
      kI.initDefault(0.0);
      kD.initDefault(0.1);
      kS.initDefault(0.0);
      kV.initDefault(0.0);
      kA.initDefault(0.0);
      kG.initDefault(0.0);
      toleranceRads.initDefault(BlockerConstants.kToleranceRads);
      kTestAngle.initDefault(BlockerConstants.kFixAngle);
    } else {
      kP.initDefault(BlockerConstants.kP);
      kI.initDefault(BlockerConstants.kI);
      kD.initDefault(BlockerConstants.kD);
      kS.initDefault(BlockerConstants.kS);
      kV.initDefault(BlockerConstants.kV);
      kA.initDefault(BlockerConstants.kA);
      kG.initDefault(BlockerConstants.kG);
      toleranceRads.initDefault(BlockerConstants.kToleranceRads);
      kTestAngle.initDefault(BlockerConstants.kFixAngle);
    }
  }

  // --- IO & Inputs ---
  private final BlockerIO io;
  private final BlockerIOInputsAutoLogged inputs = new BlockerIOInputsAutoLogged();
  private final BlockerIOOutputs outputs = new BlockerIOOutputs();

  // --- Physical joint (Geoffrey) ---
  private final PhysicalJoint.kinematics kinematicsData = new PhysicalJoint.kinematics();
  private PhysicalJoint base = PhysicalJoint.ground;

  // --- State ---
  public enum BlockerGoal {
    IDLE, // hold / coast
    OPEN, // retracted, out of the way
    BLOCKING, // deployed to block the ball path
    HALFOPEN,
    ZEROING, // move to the initial/zero angle
    CLOSE, // move to the initial/zero angle
    PUSHDOWN, // pushing down on the ball to shoot faster
    TEST // read the tunable test angle
  }

  @Getter @Setter @AutoLogOutput private BlockerGoal goal = BlockerGoal.CLOSE;

  @Getter @AutoLogOutput private boolean zeroed = true;

  @Getter
  @Accessors(fluent = true)
  @AutoLogOutput
  private boolean atGoal = false;

  private final Arm arm;

  // --- Alerts ---
  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert motorDisconnectedAlert =
      new Alert("Blocker motor disconnected!", Alert.AlertType.kWarning);

  public Blocker(BlockerIO io, Arm arm) {
    this.io = io;
    io.setPID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get(), kG.get());
    this.arm = arm;
  }

  @Override
  public void updateInputsPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Blocker", inputs);
    motorDisconnectedAlert.set(!motorConnectedDebouncer.calculate(inputs.connected));
    updateTunables();
    updateKinematics();
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled() || !zeroed) {
      outputs.mode = BlockerIOOutputMode.COAST;
      atGoal = false;
    } else {
      switch (goal) {
        case IDLE -> {
          outputs.mode = BlockerIOOutputMode.BRAKE;
          atGoal = true;
        }
        case OPEN -> runPosition(BlockerConstants.kOpenAngle);
        case HALFOPEN -> runPosition(BlockerConstants.kHalfOpenAngle);
        case BLOCKING -> runPosition(BlockerConstants.kBlockingAngle);
        case ZEROING -> runPosition(BlockerConstants.kInitialAngle);
        case CLOSE -> {
          if (arm.error >= Math.toRadians(5)) {
            runPosition(BlockerConstants.kAvoidCollisionAngle);
          } else {
            runPosition(BlockerConstants.kInitialAngle);
          }
        }
        case PUSHDOWN -> {
          outputs.mode = BlockerIOOutputMode.VOLTAGE;
          outputs.voltageOut = -1.0; // push down with 1 volt
          atGoal = true;
        }
        case TEST -> runPosition(kTestAngle.get());
      }
    }
  }

  @Override
  public void executePeriodic() {
    Logger.recordOutput("Blocker/Mode", outputs.mode);
    Logger.recordOutput("Blocker/Zeroed", zeroed);
    io.applyOutputs(outputs);
    Robot.batteryLogger.reportCurrentUsage(
        "Blocker", false, inputs.connected ? inputs.supplyCurrentAmps : 0.0);
  }

  /** Command a closed-loop angle (radians). */
  public void runPosition(double targetAngleRads) {
    runPosition(targetAngleRads, 0.0, 0.0, 0.0);
  }

  public void runPosition(
      double targetAngleRads,
      double targetVelocityRadsPerSec,
      double targetAccelerationRadsPerSecSq,
      double feedforwardAmps) {
    double clamped =
        MathUtil.clamp(targetAngleRads, BlockerConstants.kMinAngle, BlockerConstants.kMaxAngle);

    outputs.mode = BlockerIOOutputMode.POSITION;
    outputs.positionRads = clamped;
    outputs.velocityRadsPerSec = targetVelocityRadsPerSec;
    outputs.accelerationRadPerSec2 = targetAccelerationRadsPerSecSq;
    outputs.feedforwardAmps = feedforwardAmps;

    atGoal = Math.abs(inputs.positionRads - clamped) <= toleranceRads.get();

    Logger.recordOutput("Blocker/Profile/GoalPositionRad", clamped);
    Logger.recordOutput("Blocker/Profile/GoalVelocityRadPerSec", targetVelocityRadsPerSec);
  }

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

  // --- Commands ---
  public Command setGoalCommand(BlockerGoal newGoal) {
    return Commands.runOnce(() -> this.goal = newGoal, this).ignoringDisable(true);
  }

  public Command resetPositionCommand() {
    return Commands.runOnce(
        () -> {
          io.resetAngle(BlockerConstants.kInitialAngle);
        });
  }

  public Command upZeroCommand() {
    return Commands.runOnce(
        () -> {
          io.resetAngle(inputs.positionRads - BlockerConstants.adjustZeroAngleRads);
        });
  }

  public Command downZeroCommand() {
    return Commands.runOnce(
        () -> {
          io.resetAngle(inputs.positionRads + BlockerConstants.adjustZeroAngleRads);
        });
  }

  public Command shootCommand() {
    return Commands.sequence(
        setGoalCommand(BlockerGoal.OPEN), new WaitCommand(1), setGoalCommand(BlockerGoal.PUSHDOWN));
  }

  public Command zeroCommand() {
    return runOnce(
            () -> {
              zeroed = true;
              this.goal = BlockerGoal.IDLE;
            })
        .ignoringDisable(true);
  }

  // --- PhysicalJoint (Geoffrey) implementation ---
  // The blocker pivots about the robot Y axis (pitch); its rotation is the measured angle.

  @Override
  public void updateKinematics() {
    kinematicsData.forwardKinematic =
        new Transform3d(new Translation3d(), new Rotation3d(0.0, -inputs.positionRads, 0.0));

    SimpleMatrix vel = kinematicsData.localVelocity;
    vel.set(4, -inputs.velocityRadsPerSec); // pitch rate about Y

    Logger.recordOutput("Blocker/forwardKinematic", kinematicsData.forwardKinematic);
  }

  @Override
  public void setBase(PhysicalJoint base) {
    this.base = base;
  }

  @Override
  public PhysicalJoint getParentJoint() {
    return base;
  }

  @Override
  public Transform3d getForwardKinematic() {
    return kinematicsData.forwardKinematic;
  }

  @Override
  public SimpleMatrix getLocalVelocity() {
    return kinematicsData.localVelocity;
  }

  @Override
  public SimpleMatrix getLocalAcceleration() {
    return kinematicsData.localAcceleration;
  }
}
