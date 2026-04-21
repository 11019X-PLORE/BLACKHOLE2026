package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerGoal;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.intake.IntakeConstants;
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
public final class SuperstructureFactory {

  // Private constructor prevents instantiation
  private SuperstructureFactory() {}

  // ==================== HELPER METHODS ====================

  /**
   * Gets the best passing target based on robot position. Returns the closer of the two passing
   * points.
   */
  private static Translation2d getBestPassingTarget(PhysicalJoint base) {
    Translation2d blueLeft = new Translation2d(1.874, 5.49);
    Translation2d blueRight = new Translation2d(1.874, 2.17);
    Translation2d left = AllianceFlipUtil.apply(blueLeft);
    Translation2d right = AllianceFlipUtil.apply(blueRight);

    Translation2d robotPos = base.getGlobalPose().getTranslation().toTranslation2d();
    return (robotPos.getDistance(left) < robotPos.getDistance(right)) ? left : right;
  }

  /** Check if all shooter subsystems are at their goals and ready to fire. */
  private static boolean isReadyToShoot(RobotContainer c) {
    return c.getFlywheel().atGoal() && c.getHood().atGoal() && c.getTurret().atGoal();
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
                indexer.setGoal(IndexerGoal.STOP);
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

    return Commands.runOnce(
            () -> {
              indexer.setGoal(IndexerGoal.STOP);
            },
            indexer)
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
            flywheel.setGoalCommand(FlywheelGoal.FIXED_VELOCITY),
            Commands.run(
                () -> {
                  indexer.setGoal(IndexerGoal.SHOOT);
                  c.getExtension().changeCapcity(-IndexerConstants.feedingBPS / 50.0);
                },
                indexer))
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
                  Translation2d targetPos =
                      AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
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
      String commandName) {
    Turret turret = c.getTurret();
    Hood hood = c.getHood();
    Flywheel flywheel = c.getFlywheel();

    return Commands.parallel(
            turret.setGoalCommand(TurretGoal.TRACKING),
            hood.setGoalCommand(HoodGoal.TRACKING),
            flywheel.setGoalCommand(FlywheelGoal.TRACKING),
            Commands.run(
                () -> {
                  Translation2d targetPos = targetSupplier.get();
                  ShooterSetpoint sp =
                      ShooterSetpoint.makeSetpoint(
                          TurretConstants.swerve2TurretStructure,
                          targetPos,
                          FieldConstants.hMax,
                          HoodConstants.kHoodMinAngle,
                          HoodConstants.kHoodMaxAngle,
                          trajectoryConfig);

                  double lookAheadTime = useLookAhead ? 0.02 : 0.0;

                  turret.runPositionFOCLogic(
                      sp.turretPositionRadians + (lookAheadTime * sp.turretVelocityRadsPerSec),
                      sp.turretVelocityRadsPerSec
                          + (lookAheadTime * sp.turretAccelerationRadsPerSecSquared),
                      sp.turretAccelerationRadsPerSecSquared,
                      0.0);
                  hood.runPositionFOCLogic(
                      TrajectoryCalculator.getHoodSetpoint(
                          sp.hoodPositionRadians + (lookAheadTime * sp.hoodVelocityRadsPerSec)),
                      sp.hoodVelocityRadsPerSec
                          + (lookAheadTime * sp.hoodAccelerationRadsPerSecSquared),
                      sp.hoodAccelerationRadsPerSecSquared,
                      0.0);

                  double radPerSec =
                      TrajectoryCalculator.getFlywheelSetpoint(
                              sp.shooterVelocityMetersPerSec
                                  + (lookAheadTime * sp.shooterAccelerationMetersPerSecSquared))
                          / FlywheelConstants.kFlywheelRadius;
                  double radPerSec2 =
                      TrajectoryCalculator.getFlywheelAcceleration(
                              sp.shooterVelocityMetersPerSec,
                              sp.shooterAccelerationMetersPerSecSquared)
                          / FlywheelConstants.kFlywheelRadius;
                  flywheel.runVelocityFOCLogic(radPerSec, radPerSec2, 0.0);

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
        "Superstructure.Shoot");
  }

  /** Pass mode - tracks passing target and shoots when ready. */
  public static Command pass(RobotContainer c) {
    return shootWithConfig(
        c,
        () -> getBestPassingTarget(TurretConstants.swerve2TurretStructure),
        TrajectoryConfig.getPassingConfig(),
        false,
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
}
