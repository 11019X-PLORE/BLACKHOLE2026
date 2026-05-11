package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerGoal;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.intake.Intake.IntakeGoal;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.led.LED.LEDState;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.Flywheel.FlywheelGoal;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.Hood.HoodGoal;
import frc.robot.subsystems.shooter.hood.HoodConstants;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.Turret.TurretGoal;
import frc.robot.subsystems.shooter.turret.TurretConstants;
import frc.robot.util.Geoffrey.PhysicalJoint;
import frc.robot.util.Geoffrey.ShooterSetpoint;
import frc.robot.util.Geoffrey.TrajectoryCalculator;
import frc.robot.util.Geoffrey.TrajectoryConfig;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.geometry.AllianceFlipUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * SuperstructureFactory is a stateless command factory class that creates coordinated,
 * multi-subsystem commands. All methods are static - no instance state. Each method returns a fresh
 * Command object composed from subsystem primitives.
 *
 * <p>All subsystems are accessed via RobotContainer getter methods.
 */
public class SuperstructureFactory {

  // Private constructor prevents instantiation
  public final LoggedTunableNumber flywheelSetpoint =
      new LoggedTunableNumber("shooter/fixedFlywheelSetpoint");
  public final LoggedTunableNumber hoodSetpoint =
      new LoggedTunableNumber("shooter/fixedHoodSetpoint");

  public static double compensation_percent = 0;

  public static double turretShootOffset = 0; // Radians

  public SuperstructureFactory() {
    hoodSetpoint.initDefault(0);
    flywheelSetpoint.initDefault(0);
  }

  // ==================== HELPER METHODS ====================

  /**
   * Gets the best passing target based on robot position. Returns the closer of the two passing
   * points.
   */
  private static Translation2d getBestPassingTarget(PhysicalJoint base) {

    Translation2d robotPos = base.getGlobalPose().getTranslation().toTranslation2d();

    // blue right
    Translation2d origin = Translation2d.kZero;
    Translation2d corner = new Translation2d(1, 1);
    Translation2d bump = new Translation2d(3, 2.2);
    // Translation2d trench = new Translation2d(5.3, 0.8);
    Translation2d trench = bump;

    origin = AllianceFlipUtil.apply(origin);

    boolean flipY = Math.abs(robotPos.getY() - origin.getY()) > FieldConstants.fieldWidth / 2.0;

    double distance_x = Math.abs(robotPos.getX() - origin.getX());

    if (distance_x < FieldConstants.fieldLength / 2.0) {
      corner = AllianceFlipUtil.apply(corner);
      return flipY ? AllianceFlipUtil.applyY(corner) : corner;
    }
    if (distance_x < 12.8) {
      bump = AllianceFlipUtil.apply(bump);
      return flipY ? AllianceFlipUtil.applyY(bump) : bump;
    }
    trench = AllianceFlipUtil.apply(trench);
    return flipY ? AllianceFlipUtil.applyY(trench) : trench;
  }

  /**
   * Gets the best passing target based on robot position. Returns the closer of the two passing
   * points.
   */
  private static Translation2d getBestVisionTarget(PhysicalJoint base) {

    Translation2d ourHub =
        AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());

    Translation2d otherHub = AllianceFlipUtil.applyX(ourHub);

    Translation2d robotPos = base.getGlobalPose().getTranslation().toTranslation2d();

    if (robotPos.getDistance(otherHub) * 2 < robotPos.getDistance(ourHub)) {
      return otherHub;
    }
    return ourHub;
  }

  /** Check if all shooter subsystems are at their goals and ready to fire. */
  private static boolean isReadyToShoot(RobotContainer c) {
    boolean ready = c.getFlywheel().atGoal() && c.getHood().atGoal() && c.getTurret().atGoal();
    Logger.recordOutput("SuperstructureFactory/isReadyToShoot", ready);
    return ready;
  }

  // ==================== IDLE / STOP COMMANDS ====================

  /**
   * Returns all shooter subsystems to idle state. Turret, hood, flywheel coast; triggers and
   * indexer stop.
   */
  public static Command idle(RobotContainer c) {
    Turret turret = c.getTurret();
    Hood hood = c.getHood();
    Flywheel flywheel = c.getFlywheel();
    Indexer indexer = c.getIndexer();

    return Commands.parallel(
            turret.setGoalCommand(TurretGoal.IDLE),
            hood.setGoalCommand(HoodGoal.IDLE),
            flywheel.setGoalCommand(FlywheelGoal.IDLE),
            indexer.setGoalCommand(IndexerGoal.STOP))
        .withName("Superstructure.Idle");
  }

  public static Command feeding(RobotContainer c) {
    Indexer indexer = c.getIndexer();

    return Commands.run(
            () -> {
              if (isReadyToShoot(c)) {
                indexer.setGoal(IndexerGoal.SHOOT);
                c.getExtension().changeCapcity(-IndexerConstants.feedingBPS / 50.0);
              } else {
                indexer.setGoal(IndexerGoal.INTAKE);
              }
            },
            indexer)
        .withName("Superstructure.StopFeeding");
  }
  /**
   * Stops triggers and indexer without changing shooter state. Used as a "release" action for
   * intake/outtake.
   */
  public static Command stopFeeding(RobotContainer c) {
    Indexer indexer = c.getIndexer();

    // return Commands.runOnce(
    //         () -> {
    //           indexer.setGoal(IndexerGoal.STOP);
    //         },
    //         indexer)
    //     .withName("Superstructure.StopFeeding");

    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  indexer.setGoal(IndexerGoal.OUTTAKE);
                },
                indexer),
            new WaitCommand(0.1),
            Commands.runOnce(
                () -> {
                  indexer.setGoal(IndexerGoal.STOP);
                },
                indexer))
        .withName("Superstructure.StopFeeding");
  }

  // ==================== INTAKE / OUTTAKE COMMANDS ====================

  /** Runs indexer in intake direction. Triggers remain stopped. */
  public static Command runIndexerIntake(RobotContainer c) {
    Indexer indexer = c.getIndexer();

    return Commands.run(
            () -> {
              indexer.setGoal(IndexerGoal.INTAKE);
              c.getExtension().changeCapcity(IntakeConstants.kIntakeBPS / 50.0);
            },
            indexer)
        .withName("Superstructure.RunIndexerIntake");
  }

  /** Runs triggers and indexer in outtake/spit direction. */
  public static Command spit(RobotContainer c) {
    Indexer indexer = c.getIndexer();

    return Commands.run(
            () -> {
              indexer.setGoal(IndexerGoal.OUTTAKE);
              c.getExtension().changeCapcity(-IntakeConstants.kIntakeBPS / 50.0);
            },
            indexer)
        .withName("Superstructure.Spit");
  }

  /** Spins up flywheel to fixed velocity and feeds game piece through. */
  public static Command shootSpit(RobotContainer c) {
    Flywheel flywheel = c.getFlywheel();
    Indexer indexer = c.getIndexer();

    return Commands.parallel(
            // flywheel.setGoalCommand(FlywheelGoal.FIXED_VELOCITY),
            Commands.run(
                () -> {
                  indexer.setGoal(IndexerGoal.SHOOT);
                  flywheel.setGoal(FlywheelGoal.FIXED_VELOCITY);
                  c.getExtension().changeCapcity(-IndexerConstants.feedingBPS / 50.0);
                },
                indexer,
                flywheel))
        .withName("Superstructure.ShootSpit");
  }

  // ==================== ACTIVE TRACKING (Pre-aim) ====================

  /**
   * Active shooting mode - turret tracks target continuously, hood zeroed, flywheel at reduced
   * speed. This is the "ready" state before committing to a full shot.
   */
  public static Command activeShooting(RobotContainer c) {
    Turret turret = c.getTurret();
    Hood hood = c.getHood();
    Flywheel flywheel = c.getFlywheel();

    return Commands.parallel(
            turret.setGoalCommand(TurretGoal.TRACKING),
            hood.setGoalCommand(HoodGoal.ZEROING),
            flywheel.setGoalCommand(FlywheelGoal.ACTIVE),
            Commands.run(
                () -> {
                  // Translation2d targetPos =
                  //
                  // AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
                  Translation2d targetPos =
                      getBestVisionTarget(TurretConstants.swerve2TurretStructure);
                  Logger.recordOutput(
                      "SuperstructureFactory/visionTarget",
                      new Pose2d(targetPos, new Rotation2d()));

                  ShooterSetpoint sp =
                      ShooterSetpoint.makeSetpoint(
                          TurretConstants.swerve2TurretStructure,
                          targetPos,
                          HoodConstants.kHoodMinAngle,
                          0);

                  turret.runPositionFOCLogic(
                      sp.turretPositionRadians,
                      sp.turretVelocityRadsPerSec,
                      sp.turretAccelerationRadsPerSecSquared,
                      0.0);
                }))
        .withName("Superstructure.ActiveShooting");
  }

  // ==================== GENERALIZED SHOOTING ====================

  /**
   * Generalized shooting command - tracks a target position with configurable trajectory. Can be
   * used for hub shooting, passing, or any other target.
   *
   * @param c RobotContainer to get subsystems from
   * @param targetSupplier Supplier for the target position (allows dynamic targets)
   * @param trajectoryConfig Trajectory configuration for ballistics calculations
   * @param useLookAhead Whether to apply look-ahead compensation for moving shots
   * @param commandName Name for the command (for logging/debugging)
   */
  public static Command shootWithConfig(
      RobotContainer c,
      Supplier<Translation2d> targetSupplier,
      TrajectoryConfig trajectoryConfig,
      boolean useLookAhead,
      double tiltingTolerance,
      String commandName) {
    Turret turret = c.getTurret();
    Hood hood = c.getHood();
    Flywheel flywheel = c.getFlywheel();

    Translation3d rot = new Translation3d(1, 0, 0).rotateBy(c.getDrive().getRotation3d());
    boolean isTilted = Math.abs(Math.atan2(rot.getY(), rot.getX())) > tiltingTolerance;

    return Commands.parallel(
            turret.setGoalCommand(TurretGoal.TRACKING),
            hood.setGoalCommand(HoodGoal.TRACKING),
            flywheel.setGoalCommand(FlywheelGoal.TRACKING),
            Commands.run(
                () -> {
                  boolean inTrench = c.getDrive().isInTrenchZone() && DriverStation.isTeleop();
                  Translation2d targetPos = targetSupplier.get();
                  Logger.recordOutput(
                      "SuperstructureFactory/shootingTarget",
                      new Pose2d(targetSupplier.get(), new Rotation2d()));
                  ShooterSetpoint sp =
                      ShooterSetpoint.makeSetpoint(
                          TurretConstants.swerve2TurretStructure,
                          targetPos,
                          FieldConstants.hMax,
                          HoodConstants.kHoodMinAngle,
                          HoodConstants.kHoodMaxAngle,
                          trajectoryConfig,
                          false);

                  double lookAheadTime = useLookAhead ? 0.02 : 0.0;

                  turret.runPositionFOCLogic(
                      sp.turretPositionRadians
                          + (lookAheadTime * sp.turretVelocityRadsPerSec)
                          + turretShootOffset,
                      sp.turretVelocityRadsPerSec
                          + (lookAheadTime * sp.turretAccelerationRadsPerSecSquared),
                      sp.turretAccelerationRadsPerSecSquared,
                      0.0);
                  if (inTrench) {
                    hood.runPositionFOCLogic(
                        (Math.PI / 2) - HoodConstants.kHoodInitialAngle, 0.0, 0.0, 0.0);
                  } else {
                    hood.runPositionFOCLogic(
                        TrajectoryCalculator.getHoodSetpoint(
                            sp.hoodPositionRadians + (lookAheadTime * sp.hoodVelocityRadsPerSec)),
                        sp.hoodVelocityRadsPerSec
                            + (lookAheadTime * sp.hoodAccelerationRadsPerSecSquared),
                        sp.hoodAccelerationRadsPerSecSquared,
                        0.0);
                  }

                  double radPerSec =
                      (1 + (compensation_percent / 100.0))
                          * TrajectoryCalculator.getFlywheelSetpoint(
                              sp.shooterVelocityMetersPerSec
                                  + (lookAheadTime * sp.shooterAccelerationMetersPerSecSquared))
                          / FlywheelConstants.kFlywheelRadius;
                  double radPerSec2 =
                      (1 + (compensation_percent / 100.0))
                          * TrajectoryCalculator.getFlywheelAcceleration(
                              sp.shooterVelocityMetersPerSec,
                              sp.shooterAccelerationMetersPerSecSquared)
                          / FlywheelConstants.kFlywheelRadius;
                  flywheel.runVelocityFOCLogic(radPerSec, radPerSec2, 0.0);

                  // turret.runPositionFOCLogic(0, 0, 0, 0);

                  // flywheel.runVelocityFOCLogic(15 / FlywheelConstants.kFlywheelRadius, 0, 0.0);

                  // hood.runPositionFOCLogic(Math.toRadians(86), 0, 0, 0);

                  Logger.recordOutput(
                      "shooterSetpoint/shooterVelocityMetersPerSec",
                      sp.shooterVelocityMetersPerSec);
                  Logger.recordOutput(
                      "shooterSetpoint/turretPositionRadians", sp.turretPositionRadians);
                  Logger.recordOutput(
                      "shooterSetpoint/turretVelocityRadsPerSec", sp.turretVelocityRadsPerSec);
                  Logger.recordOutput(
                      "shooterSetpoint/turretAccelerationRadsPerSecSquared",
                      sp.turretAccelerationRadsPerSecSquared);
                  Logger.recordOutput(
                      "shooterSetpoint/hoodPositionRadians", sp.hoodPositionRadians);
                  Logger.recordOutput(
                      "shooterSetpoint/hoodVelocityRadsPerSec", sp.hoodVelocityRadsPerSec);
                  Logger.recordOutput(
                      "shooterSetpoint/hoodAccelerationRadsPerSecSquared",
                      sp.hoodAccelerationRadsPerSecSquared);
                }))
        .withName(commandName);
  }

  /**
   * Full shooting mode - all shooter subsystems track hub target. Feeds game piece when ready to
   * shoot.
   */
  public static Command shoot(RobotContainer c) {
    return shootWithConfig(
        c,
        () -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d()),
        TrajectoryConfig.getHubConfig(),
        true,
        0.1,
        "Superstructure.Shoot");
  }

  /** Pass mode - tracks passing target and shoots when ready. */
  public static Command pass(RobotContainer c) {
    return shootWithConfig(
        c,
        () -> getBestPassingTarget(TurretConstants.swerve2TurretStructure),
        TrajectoryConfig.getPassingConfig(),
        false,
        0.1,
        "Superstructure.Pass");
  }

  /** Fixed-angle shooting - turret and hood go to fixed positions, flywheel at fixed velocity. */
  public static Command shootFixed(RobotContainer c) {
    Turret turret = c.getTurret();
    Hood hood = c.getHood();
    Flywheel flywheel = c.getFlywheel();
    Indexer indexer = c.getIndexer();

    return Commands.parallel(
            turret.setGoalCommand(TurretGoal.FIXED_ANGLE),
            hood.setGoalCommand(HoodGoal.FIXED_ANGLE),
            flywheel.setGoalCommand(FlywheelGoal.FIXED_VELOCITY),
            Commands.run(
                () -> {
                  if (isReadyToShoot(c)) {
                    indexer.setGoal(IndexerGoal.SHOOT);
                  } else {
                    indexer.setGoal(IndexerGoal.STOP);
                  }
                }))
        .withName("Superstructure.ShootFixed");
  }

  // ==================== SAFETY / TRENCH ====================

  /** Trench mode - hood zeroed for clearance, turret idle. Safety override state. */
  public static Command trench(RobotContainer c) {
    Turret turret = c.getTurret();
    Hood hood = c.getHood();
    Flywheel flywheel = c.getFlywheel();
    Indexer indexer = c.getIndexer();

    return Commands.parallel(
            turret.setGoalCommand(TurretGoal.IDLE),
            hood.setGoalCommand(HoodGoal.ZEROING),
            flywheel.setGoalCommand(FlywheelGoal.TRACKING),
            Commands.runOnce(
                () -> {
                  indexer.setGoal(IndexerGoal.STOP);
                },
                indexer))
        .withName("Superstructure.Trench");
  }

  // ==================== TEST MODE ====================

  /** Test mode - turret in test mode, feeds when ready. */
  public static Command test(RobotContainer c) {
    Turret turret = c.getTurret();
    Indexer indexer = c.getIndexer();

    return Commands.parallel(
            turret.setGoalCommand(TurretGoal.TEST),
            Commands.run(
                () -> {
                  if (isReadyToShoot(c)) {
                    indexer.setGoal(IndexerGoal.SHOOT);
                  } else {
                    indexer.setGoal(IndexerGoal.STOP);
                  }
                },
                indexer))
        .withName("Superstructure.Test");
  }

  public static Command ledMonitor(RobotContainer c) {
    return Commands.run(
            () -> {
              boolean inTrench = c.getDrive().isInTrenchZone() && DriverStation.isTeleop();

              // 1. 自动阶段 (强制最高优先级)
              if (DriverStation.isAutonomous()) {
                c.getLED().setGoal(LEDState.AUTO);
                return;
              }

              // 2. TRENCH 保护 (物理安全，最高优先级)
              if (inTrench) {
                c.getLED().setGoal(LEDState.TRENCH);
                return;
              }

              // 3. 发射与瞄准状态判断
              if (c.getFlywheel().getGoal() == FlywheelGoal.TRACKING
                  || c.getFlywheel().getGoal() == FlywheelGoal.FIXED_VELOCITY
                  || c.getFlywheel().getGoal() == FlywheelGoal.PASSING) {

                if (isReadyToShoot(c)) {
                  if (c.getIndexer().getGoal() == IndexerGoal.SHOOT) {
                    c.getLED().setGoal(LEDState.SHOOTING); // 正在开火
                  } else {
                    c.getLED().setGoal(LEDState.READY_TO_SHOOT); // 瞄准完毕，随时可打
                  }
                } else {
                  c.getLED().setGoal(LEDState.INITIAL); // 正在追踪瞄准中
                }
                return;
              }

              // 4. 吸球吐球动作
              if (c.getIntake().getGoal() == IntakeGoal.INTAKE) {
                c.getLED().setGoal(LEDState.INTAKING);
                return;
              }

              if (c.getIntake().getGoal() == IntakeGoal.OUTTAKE) {
                c.getLED().setGoal(LEDState.OUTTAKE);
              }

              // 5. STOW (收起状态)
              if (c.getIntake().getGoal() == IntakeGoal.STOW) {
                c.getLED().setGoal(LEDState.INTAKE_STOWED);
                return;
              }

              // 6. 默认状态 (IDLE)
              c.getLED().setGoal(LEDState.INITIAL);
            },
            c.getLED())
        .ignoringDisable(true)
        .withName("Superstructure.LEDMonitor");
  }
}
