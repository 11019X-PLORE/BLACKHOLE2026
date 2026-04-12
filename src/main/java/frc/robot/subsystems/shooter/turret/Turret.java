package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.hood.HoodConstants;
import frc.robot.subsystems.shooter.turret.Turret.TurretGoal;
import frc.robot.subsystems.shooter.turret.TurretIO.TurretIOOutputMode;
import frc.robot.subsystems.shooter.turret.TurretIO.TurretIOOutputs;
import frc.robot.util.EqualsUtil;
import frc.robot.util.FullSubsystem;
import frc.robot.util.Geoffrey.PhysicalJoint;
import frc.robot.util.Geoffrey.ShooterSetpoint;
import frc.robot.util.Geoffrey.TrajectoryConfig;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.geometry.AllianceFlipUtil;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;
import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends FullSubsystem implements PhysicalJoint {
  private static final double trackOverlapMargin = Units.degreesToRadians(10);
  private static final double trackMinAngle = TurretConstants.kTurretMinAngle - trackOverlapMargin;
  private static final double trackMaxAngle = TurretConstants.kTurretMaxAngle + trackOverlapMargin;

  private static final LoggedTunableNumber testVelocity =
      new LoggedTunableNumber("turret/TestVelocity");
  public static final LoggedTunableNumber kP = new LoggedTunableNumber("turret/PID/kP");
  public static final LoggedTunableNumber kI = new LoggedTunableNumber("turret/PID/kI");
  public static final LoggedTunableNumber kD = new LoggedTunableNumber("turret/PID/kD");
  public static final LoggedTunableNumber kA = new LoggedTunableNumber("turret/PID/kA");
  public static final LoggedTunableNumber kV = new LoggedTunableNumber("turret/PID/kV");
  public static final LoggedTunableNumber kS = new LoggedTunableNumber("turret/PID/kS");
  public static final LoggedTunableNumber kG = new LoggedTunableNumber("turret/PID/kG");
  private static final LoggedTunableNumber kFixAngle = new LoggedTunableNumber("turret/kFixAngle");
  public static final LoggedTunableNumber toleranceDeg =
      new LoggedTunableNumber("turret/ToleranceDeg");

  static {
    if (Robot.isSimulation()) {
      kP.initDefault(35);
      kI.initDefault(0.0);
      kD.initDefault(0.1);
      kS.initDefault(0.0);
      kV.initDefault(0.0);
      kA.initDefault(0.0);
      kG.initDefault(0.0);
      toleranceDeg.initDefault(1.0);
      kFixAngle.initDefault(90.0);
      testVelocity.initDefault(TurretConstants.kMaxAngularVelocityRadpersec);
    } else {
      kP.initDefault(TurretConstants.kP);
      kI.initDefault(TurretConstants.kI);
      kD.initDefault(TurretConstants.kD);
      kS.initDefault(TurretConstants.kS);
      kV.initDefault(TurretConstants.kV);
      kA.initDefault(TurretConstants.kA);
      kG.initDefault(TurretConstants.kG);
      toleranceDeg.initDefault(TurretConstants.kTurretToleranceDeg);
      kFixAngle.initDefault(TurretConstants.kTurretFixAngle);
      testVelocity.initDefault(TurretConstants.kVelocityRadPerSec);
    }
  }

  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final TurretIOOutputs outputs = new TurretIOOutputs();

  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> chassisSpeedSupplier;

  public enum TurretGoal {
    IDLE,
    TRACKING,
    FIXED_ANGLE,
    PASSING,
    ZEROING,
    TEST
  }

  @Getter @Setter @AutoLogOutput private TurretGoal goal = TurretGoal.IDLE;

  private double lastGoalAngle = 0.0;

  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Alert disconnected =
      new Alert("Turret motor disconnected!", Alert.AlertType.kWarning);

  @Getter @AutoLogOutput private boolean turretZeroed = false;

  @Getter
  @Accessors(fluent = true)
  @AutoLogOutput
  private boolean atGoal = false;

  private final Translation2d robotToTurret;
  private Pose2d turretPose = new Pose2d();
  private final PhysicalJoint.kinematics kinematicsData = new PhysicalJoint.kinematics();
  private PhysicalJoint base = null;

  public Turret(
      TurretIO io,
      Translation2d robotToTurret,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> chassisSpeedSupplier) {
    this.io = io;
    this.robotToTurret = robotToTurret;
    this.poseSupplier = poseSupplier;
    this.chassisSpeedSupplier = chassisSpeedSupplier;
    io.setPID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get(), kG.get());
  }

  @Override
  public void updateInputsPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("turret", inputs);
    disconnected.set(!motorConnectedDebouncer.calculate(inputs.turretMotorConnected));
    updateTunables();
    calculateTurretPose();
    updateKinematics();
  }

  @Override
  public void periodic() {

    if (DriverStation.isDisabled() || !turretZeroed) {
      outputs.mode = TurretIOOutputMode.COAST;
      outputs.velocityRadsPerSec = 0.0;
      atGoal = false;
      // 禁用时强制同步位置，防止下次启用时猛跳
      lastGoalAngle = inputs.positionRads;
    } else {
      switch (goal) {
        case IDLE -> {
          outputs.mode = TurretIOOutputMode.COAST;
          outputs.velocityRadsPerSec = 0.0;
          atGoal = true;
        }
        case TRACKING -> {
          // Translation2d targetPos =
          //     AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
          // ShooterSetpoint sp =
          //     ShooterSetpoint.makeSetpoint(
          //         TurretConstants.swerve2TurretStructure,
          //         targetPos,
          //         FieldConstants.hMax,
          //         HoodConstants.kHoodMinAngle,
          //         HoodConstants.kHoodMaxAngle,
          //         TrajectoryConfig.getHubConfig());

          // runPositionFOCLogic(
          //     sp.turretPositionRadians,
          //     sp.turretVelocityRadsPerSec,
          //     sp.turretAccelerationRadsPerSecSquared,
          //     0.0);
        }
        case PASSING -> {
          // Translation2d passTarget = getBestPassingTarget();
          // ShooterSetpoint sp =
          //     ShooterSetpoint.makeSetpoint(
          //         TurretConstants.swerve2TurretStructure,
          //         passTarget,
          //         FieldConstants.hMax,
          //         HoodConstants.kHoodMinAngle,
          //         HoodConstants.kHoodMaxAngle,
          //         TrajectoryConfig.getPassingConfig());
          // runPositionFOCLogic(
          //     sp.turretPositionRadians,
          //     sp.turretVelocityRadsPerSec,
          //     sp.turretAccelerationRadsPerSecSquared,
          //     0.0);
        }
        case FIXED_ANGLE -> {
          var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
          Rotation2d targetFieldAngle =
              (alliance == Alliance.Red)
                  ? Rotation2d.fromDegrees(-180.0)
                  : Rotation2d.fromDegrees(0.0);
          double compensateVel = -chassisSpeedSupplier.get().omegaRadiansPerSecond;
          runPositionFOCLogic(targetFieldAngle.getRadians(), compensateVel, 0.0, 0.0);
        }
        case ZEROING -> {
          Rotation2d targetFieldAngle = Rotation2d.fromDegrees(-180.0);
          double compensateVel = -chassisSpeedSupplier.get().omegaRadiansPerSecond;
          runPositionFOCLogic(targetFieldAngle.getRadians(), compensateVel, 0.0, 0.0);
        }
        case TEST -> {
          Rotation2d targetFieldAngle = Rotation2d.fromDegrees(-90.0);
          double compensateVel = -chassisSpeedSupplier.get().omegaRadiansPerSecond;
          runPositionFOCLogic(targetFieldAngle.getRadians(), compensateVel, 0.0, 0.0);
        }
      }
    }
  }

  @Override
  public void executePeriodic() {
    io.applyOutputs(outputs);
    Logger.recordOutput("turret/atGoal", atGoal);
    Logger.recordOutput("turret/mode", outputs.mode);
  }

  public void runPositionFOCLogic(
      double targetFieldAngleRad, double targetVel, double targetAccel, double feedforwardAmps) {
    double targetRads = targetFieldAngleRad;
    // 1. 寻找 [-270, 90] 物理限位内最近的等效点
    boolean hasBestAngle = false;
    double bestAngle = 0;
    for (int i = -2; i < 3; i++) {
      double potentialSetpoint = targetRads + (i * 2.0 * Math.PI);
      if (potentialSetpoint >= TurretConstants.kTurretMinAngle
          && potentialSetpoint <= TurretConstants.kTurretMaxAngle) {
        if (!hasBestAngle
            || Math.abs(lastGoalAngle - potentialSetpoint) < Math.abs(lastGoalAngle - bestAngle)) {
          bestAngle = potentialSetpoint;
          hasBestAngle = true;
        }
      }
    }

    // 强制 Clamp 到物理边界
    if (!hasBestAngle) {
      bestAngle =
          MathUtil.clamp(
              targetRads, TurretConstants.kTurretMinAngle, TurretConstants.kTurretMaxAngle);
    }

    lastGoalAngle = bestAngle;

    outputs.mode = TurretIOOutputMode.POSITION_FOC;
    outputs.positionRads = bestAngle;
    outputs.velocityRadsPerSec = targetVel;
    outputs.accelerationRadPerSec2 = targetAccel;
    outputs.feedforwardAmps = feedforwardAmps;

    atGoal = EqualsUtil.epsilonEquals(bestAngle, inputs.positionRads, toleranceDeg.get());

    double goalStateAngle =
        MathUtil.clamp(bestAngle, TurretConstants.kTurretMinAngle, TurretConstants.kTurretMaxAngle);
    Logger.recordOutput("turret/GoalPositionRad", bestAngle);
    Logger.recordOutput("turret/SetpointPositionRad", goalStateAngle);
    Logger.recordOutput("turret/GoalVelocityRadPerSec", targetVel);
    Logger.recordOutput("turret/GoalAccelerationRadPerSec2", targetAccel);
  }

  private void updateTunables() {
    if (kP.hasChanged(hashCode())
        || kD.hasChanged(hashCode())
        || kA.hasChanged(hashCode())
        || kV.hasChanged(hashCode())
        || kS.hasChanged(hashCode())
        || kI.hasChanged(hashCode())
        || kG.hasChanged(hashCode())) {
      io.setPID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get(), kG.get());
    }
  }

  public void zero() {
    turretZeroed = true;
    lastGoalAngle = inputs.positionRads;
  }

  public Command setGoalCommand(TurretGoal newGoal) {
    return Commands.runOnce(() -> this.goal = newGoal, this);
  }

  public Command zeroCommand() {
    return runOnce(
            () -> {
              turretZeroed = true;
              lastGoalAngle = inputs.positionRads;
              this.goal = TurretGoal.IDLE;
            })
        .ignoringDisable(true);
  }

  public void calculateTurretPose() {
    if (turretZeroed) {
      turretPose =
          poseSupplier
              .get()
              .transformBy(
                  new Transform2d(robotToTurret, Rotation2d.fromRadians(inputs.positionRads)));
    }
  }

  public Pose2d getTurretpose() {
    return turretPose;
  }

  public double getPosition() {
    return inputs.positionRads;
  }

  public double getVelocity() {
    return inputs.velocityRadsPerSec;
  }

  public Translation2d getRobotToTurret() {
    return robotToTurret;
  }

  public Pose3d getCameraPoseRobotSpace() {
    Pose3d turretBase =
        new Pose3d(
            robotToTurret.getX(),
            robotToTurret.getY(),
            0.407,
            new Rotation3d(0, 0, Math.PI)); // 炮塔中心相对于机器人中心的位姿，TODO: 测量确认
    Rotation3d turretRotation = new Rotation3d(0.0, 0.0, inputs.positionRads);
    Transform3d cameraOnTurret = TurretConstants.kCameraonTurretoffset;
    return turretBase
        .transformBy(new Transform3d(new Translation3d(), turretRotation))
        .transformBy(cameraOnTurret);
  }

  public Rotation2d getFieldRotation() {
    return poseSupplier.get().getRotation().plus(new Rotation2d(getPosition()));
  }

  public void launchFuel() {
    io.launchFuel();
  }

  // TODO call this in robot container after creating turret and drive, to set up the kinematics
  // chain
  public void setBase(PhysicalJoint base) {
    this.base = base;
  }

  @Override
  public void updateKinematics() {
    kinematicsData.forwardKinematic =
        new Transform3d(new Translation3d(), new Rotation3d(0, 0, getPosition()));
    Logger.recordOutput("Turret/forwardKinematic", kinematicsData.forwardKinematic);

    kinematicsData.localVelocity = new SimpleMatrix(6, 1);
    kinematicsData.localVelocity.set(5, 0, getVelocity());
    kinematicsData.localAcceleration = new SimpleMatrix(6, 1); // NO need to know,
  }
  ;

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
