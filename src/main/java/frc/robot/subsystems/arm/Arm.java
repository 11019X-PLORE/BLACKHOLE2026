package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmIO.ArmIOOutputMode;
import frc.robot.subsystems.arm.ArmIO.ArmIOOutputs;
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
 * The Arm both deploys the intake (angle to the floor) and controls the shooting angle. It is a
 * single jointed arm with closed-loop position control, and is part of the Geoffrey {@link
 * PhysicalJoint} chain so downstream mechanisms (e.g. the flywheel muzzle) can be located on the
 * field.
 */
public class Arm extends FullSubsystem implements PhysicalJoint {

  // --- Tunables ---
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Arm/kP");
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Arm/kI");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Arm/kD");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Arm/kS");
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Arm/kV");
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Arm/kA");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Arm/kG");
  private static final LoggedTunableNumber kTestAngleDeg =
      new LoggedTunableNumber("Arm/kTestAngleDeg");
  private static final LoggedTunableNumber toleranceRads =
      new LoggedTunableNumber("Arm/ToleranceRads");

  static {
    if (Robot.isSimulation()) {
      kP.initDefault(5.0);
      kI.initDefault(0.0);
      kD.initDefault(0.1);
      kS.initDefault(0.0);
      kV.initDefault(0.0);
      kA.initDefault(0.0);
      kG.initDefault(0.0);
      toleranceRads.initDefault(ArmConstants.kToleranceRads);
      kTestAngleDeg.initDefault(Math.toDegrees(ArmConstants.kFixAngle));
    } else {
      kP.initDefault(ArmConstants.kP);
      kI.initDefault(ArmConstants.kI);
      kD.initDefault(ArmConstants.kD);
      kS.initDefault(ArmConstants.kS);
      kV.initDefault(ArmConstants.kV);
      kA.initDefault(ArmConstants.kA);
      kG.initDefault(ArmConstants.kG);
      toleranceRads.initDefault(ArmConstants.kToleranceRads);
      kTestAngleDeg.initDefault(Math.toDegrees(ArmConstants.kFixAngle));
    }
  }

  // --- IO & Inputs ---
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private final ArmIOOutputs outputs = new ArmIOOutputs();

  // --- Physical joint (Geoffrey) ---
  private final PhysicalJoint.kinematics kinematicsData = new PhysicalJoint.kinematics();
  private PhysicalJoint base = PhysicalJoint.ground;

  // --- State ---
  public enum ArmGoal {
    IDLE, // hold / coast
    INTAKE, // deployed to the floor for intaking
    STOW, // folded up
    PASSING,
    SHOOTING, // controlling the shooting angle (uses fixedAngleRads)
    TRACKING,
    ZEROING, // move to the initial/zero angle
    TEST // read the tunable test angle
  }

  @Getter @Setter @AutoLogOutput private ArmGoal goal = ArmGoal.IDLE;

  /** Setpoint used by the SHOOTING goal. Set this from your aiming logic. */
  @Setter private double fixedAngleRads = ArmConstants.kShootAngle;

  /** Velocity feedforward (rad/s) applied while in the SHOOTING goal. */
  @Setter private double fixedVelocityRadsPerSec = 0.0;

  /** Acceleration feedforward (rad/s^2) applied while in the SHOOTING goal. */
  @Setter private double fixedAccelerationRadsPerSec2 = 0.0;

  @Getter @AutoLogOutput private boolean zeroed = true;

  public double targetAngle = 0;

  @Getter
  @Accessors(fluent = true)
  @AutoLogOutput
  private boolean atGoal = false;

  public double error = 0;

  // --- Alerts ---
  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert motorDisconnectedAlert =
      new Alert("Arm motor disconnected!", Alert.AlertType.kWarning);

  public Arm(ArmIO io) {
    this.io = io;
    io.setPID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get(), kG.get());
  }

  @Override
  public void updateInputsPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
    motorDisconnectedAlert.set(!motorConnectedDebouncer.calculate(inputs.connected));
    updateTunables();
    updateKinematics();
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled() || !zeroed) {
      outputs.mode = ArmIOOutputMode.COAST;
      atGoal = false;
    } else {
      switch (goal) {
        case IDLE -> {
          outputs.mode = ArmIOOutputMode.BRAKE;
          atGoal = true;
          error = 0;
        }
        case INTAKE -> runPosition(ArmConstants.kIntakeAngle);
        case STOW -> runPosition(ArmConstants.kStowAngle);
        case SHOOTING -> runPosition(
            fixedAngleRads, fixedVelocityRadsPerSec, fixedAccelerationRadsPerSec2, 0.0);
        case PASSING -> runPosition(ArmConstants.kPassingAngle);
        case ZEROING -> runPosition(ArmConstants.kInitialAngle);
        case TEST -> runPosition(Math.toRadians(kTestAngleDeg.get()));
        case TRACKING -> runPosition(targetAngle);
      }
    }
  }

  @Override
  public void executePeriodic() {
    Logger.recordOutput("Arm/Mode", outputs.mode);
    Logger.recordOutput("Arm/Zeroed", zeroed);
    io.applyOutputs(outputs);
    Robot.batteryLogger.reportCurrentUsage(
        "Arm", false, inputs.connected ? inputs.supplyCurrentAmps : 0.0);
  }

  /** Command a closed-loop angle (radians from horizontal). */
  public void runPosition(double targetAngleRads) {
    runPosition(targetAngleRads, 0.0, 0.0, 0.0);
  }

  public void runPosition(
      double targetAngleRads,
      double targetVelocityRadsPerSec,
      double targetAccelerationRadsPerSecSq,
      double feedforwardAmps) {
    double clamped =
        MathUtil.clamp(targetAngleRads, ArmConstants.kMinAngle, ArmConstants.kMaxAngle);

    outputs.mode = ArmIOOutputMode.POSITION;
    outputs.positionRads = clamped;
    outputs.velocityRadsPerSec = targetVelocityRadsPerSec;
    outputs.accelerationRadPerSec2 = targetAccelerationRadsPerSecSq;
    outputs.feedforwardAmps = feedforwardAmps;

    error = clamped - inputs.positionRads;
    atGoal = Math.abs(error) <= toleranceRads.get();

    Logger.recordOutput("Arm/Profile/GoalPositionRad", clamped);
    Logger.recordOutput("Arm/Profile/GoalVelocityRadPerSec", targetVelocityRadsPerSec);
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
  public Command setGoalCommand(ArmGoal newGoal) {
    return Commands.runOnce(() -> this.goal = newGoal, this).ignoringDisable(true);
  }

  // --- Commands ---
  public Command calibrateZeroCommand(ArmGoal newGoal) {
    return Commands.sequence(
        new InstantCommand(
            () -> {
              targetAngle = ArmConstants.kInitialAngle + ArmConstants.backlashAngle;
            }),
        setGoalCommand(ArmGoal.TRACKING),
        new WaitCommand(0.2),
        setGoalCommand(ArmGoal.IDLE));
  }

  public Command zeroCommand() {
    return runOnce(
            () -> {
              zeroed = true;
              this.goal = ArmGoal.IDLE;
            })
        .ignoringDisable(true);
  }

  public boolean isAtGoal() {
    return atGoal;
  }

  // --- PhysicalJoint (Geoffrey) implementation ---
  // The arm pivots about the robot Y axis (pitch); its rotation is the measured angle.

  @Override
  public void updateKinematics() {
    kinematicsData.forwardKinematic =
        new Transform3d(new Translation3d(), new Rotation3d(0.0, -inputs.positionRads, 0.0));

    SimpleMatrix vel = kinematicsData.localVelocity;
    vel.set(4, -inputs.velocityRadsPerSec); // pitch rate about Y

    Logger.recordOutput("Arm/forwardKinematic", kinematicsData.forwardKinematic);
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
