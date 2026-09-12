// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandFactories;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmGoal;
import frc.robot.subsystems.blocker.Blocker.BlockerGoal;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.Flywheel.FlywheelGoal;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerGoal;
import frc.robot.util.Geoffrey.ShooterSetpoint;
import frc.robot.util.geometry.AllianceFlipUtil;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Stateless command factory for the offseason superstructure.
 *
 * <p>Every method is static and returns a fresh {@link Command} composed from subsystem primitives.
 * Subsystems are reached through the {@link RobotContainer}'s public fields ({@code c.drive},
 * {@code c.arm}, ...).
 *
 * <p>Because this robot has no turret, the "aim" is performed by rotating the whole chassis. The
 * shooting commands therefore run two things in parallel:
 *
 * <ol>
 *   <li>a heading-servo drive command ({@link DriveCommands#aimAtTarget}) that points the chassis
 *       at the target using PID + a yaw-velocity feedforward, and
 *   <li>a per-loop update of the arm angle and flywheel velocity from a {@link ShooterSetpoint},
 *       feeding the indexer once everything is on-target.
 * </ol>
 */
public final class CommandFactory {

  private CommandFactory() {}

  /** Heading error (rad) under which the chassis is considered aimed. */
  private static final double kHeadingToleranceRads = Units.degreesToRadians(5.0);

  /**
   * Extra percent added to the computed flywheel speed to overcome real-world energy losses (0 = no
   * compensation). Exposed as a plain static so it can be tweaked at runtime.
   */
  public static double compensationPercent = 0.0;

  // ==================== HELPERS ====================

  /** The hub we are shooting into, alliance-flipped, as a 2d field point. */
  private static Translation2d hubTarget() {
    return AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
  }

  /** Apply the runtime speed compensation to a flywheel velocity (rad/s). */
  private static double withCompensation(double velocityRadsPerSec) {
    return velocityRadsPerSec * (1.0 + (compensationPercent / 100.0));
  }

  /** True when the chassis heading is within tolerance of the desired shot heading. */
  @AutoLogOutput
  private static boolean headingAligned(Drive drive, ShooterSetpoint sp) {

    double error =
        MathUtil.angleModulus(drive.getRotation().getRadians() - sp.yawPositionRadians());
    Logger.recordOutput("CommandFactory/headingError(deg)", Math.toDegrees(error));
    return Math.abs(error) <= kHeadingToleranceRads;
  }

  /** True when the arm, flywheel and chassis heading are all ready to fire. */
  private static boolean readyToShoot(RobotContainer c, ShooterSetpoint sp) {
    boolean ready =
        // sp.valid() &&
        c.arm.isAtGoal() && c.flywheel.atGoal() && headingAligned(c.drive, sp);
    Logger.recordOutput("CommandFactory/readyToShoot", ready);
    return ready;
  }

  // ==================== IDLE / STOP ====================

  /** Everything to a safe idle: flywheel coasts, indexer stops, arm stows. */
  public static Command idle(RobotContainer c) {
    return Commands.parallel(
            c.arm.setGoalCommand(ArmGoal.STOW),
            c.flywheel.setGoalCommand(FlywheelGoal.IDLE),
            c.indexer.setGoalCommand(IndexerGoal.STOP))
        .withName("Superstructure.Idle");
  }

  // ==================== INTAKE / OUTTAKE ====================

  /** Deploy the arm and run the flywheel + indexer inward to intake a ball. */
  public static Command intake(RobotContainer c) {
    return Commands.parallel(
            c.arm.setGoalCommand(ArmGoal.INTAKE),
            c.flywheel.setGoalCommand(FlywheelGoal.INTAKE),
            c.indexer.setGoalCommand(IndexerGoal.INTAKE))
        .withName("Superstructure.Intake");
  }

  /** Reverse the flywheel + indexer to spit a ball back out. */
  public static Command outtake(RobotContainer c) {
    return Commands.parallel(
            c.flywheel.setGoalCommand(FlywheelGoal.OUTTAKE),
            c.indexer.setGoalCommand(IndexerGoal.OUTTAKE))
        .withName("Superstructure.Outtake");
  }

  /** Fold the arm up and stop the handling. */
  public static Command stow(RobotContainer c) {
    return Commands.parallel(
            c.arm.setGoalCommand(ArmGoal.STOW),
            c.flywheel.setGoalCommand(FlywheelGoal.IDLE),
            c.indexer.setGoalCommand(IndexerGoal.STOP))
        .withName("Superstructure.Stow");
  }

  // ==================== BLOCKER ====================

  /** Deploy the blocker while scheduled; opens again when the command ends. */
  public static Command block(RobotContainer c) {
    return c.blocker
        .setGoalCommand(BlockerGoal.BLOCKING)
        .andThen(Commands.idle(c.blocker))
        .finallyDo(() -> c.blocker.setGoal(BlockerGoal.OPEN))
        .withName("Superstructure.Block");
  }

  // ==================== SHOOTING ====================

  /**
   * Generalized shooting command. Aims the chassis at {@code targetSupplier} with a profiled
   * heading PID + yaw feedforward while continuously setting the flywheel velocity from a {@link
   * ShooterSetpoint}. The arm holds its fixed shooting angle. Feeds the ball once the arm, flywheel
   * and heading are all on-target. The driver keeps translational control through the joystick
   * suppliers.
   *
   * @param c robot container
   * @param targetSupplier field-relative target to shoot at (alliance flipping already applied)
   * @param xSupplier driver forward/back translation input (field relative)
   * @param ySupplier driver left/right translation input (field relative)
   * @param name command name for logging
   */
  public static Command shootAtTarget(
      RobotContainer c,
      Supplier<Translation2d> targetSupplier,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      String name) {

    Drive drive = c.drive;
    Arm arm = c.arm;
    Flywheel flywheel = c.flywheel;
    Indexer indexer = c.indexer;

    // Shared, single-element holder updated by the mechanism loop and read by the aim command.
    final ShooterSetpoint[] holder = {ShooterSetpoint.kEmpty};

    Command aim =
        DriveCommands.aimAtTarget(
            drive,
            xSupplier,
            ySupplier,
            () -> holder[0].yaw(),
            () -> holder[0].yawVelocityRadsPerSec());

    Command mechanism =
        Commands.run(
            () -> {
              Translation2d target = targetSupplier.get();
              ShooterSetpoint sp = ShooterSetpoint.makeSetpoint(drive, target);
              holder[0] = sp;

              // Arm: hold the fixed shooting angle (no per-shot angle solve on this robot).
              // arm.setGoal(ArmGoal.SHOOTING);

              arm.targetAngle = sp.pitchAngleRads();
              arm.setGoal(ArmGoal.TRACKING);

              // Flywheel: track the distance-based velocity from the real-data fit.
              flywheel.setTrackingVelocityRadsPerSec(
                  withCompensation(sp.shooterVelocityRadsPerSec()));
              flywheel.setTrackingAccelerationRadsPerSec2(0.0);
              flywheel.setGoal(FlywheelGoal.TRACKING);

              // Feed only when the whole shot is dialed in.
              indexer.setGoal(readyToShoot(c, sp) ? IndexerGoal.SHOOT : IndexerGoal.SLOWFORWARD);

              Logger.recordOutput(
                  "CommandFactory/shootingTarget", new Pose2d(target, Rotation2d.kZero));
              Logger.recordOutput("CommandFactory/distanceMeters", sp.distanceMeters());
              Logger.recordOutput(
                  "CommandFactory/shooterVelocityRadsPerSec", sp.shooterVelocityRadsPerSec());
              Logger.recordOutput("CommandFactory/timeOfFlightSeconds", sp.timeOfFlightSeconds());
              Logger.recordOutput("CommandFactory/yawPositionRadians", sp.yawPositionRadians());
              Logger.recordOutput(
                  "CommandFactory/yawVelocityRadsPerSec", sp.yawVelocityRadsPerSec());
            },
            arm,
            flywheel,
            indexer);

    return Commands.parallel(aim, mechanism)
        .beforeStarting(() -> holder[0] = ShooterSetpoint.makeSetpoint(drive, targetSupplier.get()))
        .finallyDo(
            () -> {
              arm.setGoal(ArmGoal.STOW);
              flywheel.setGoal(FlywheelGoal.IDLE);
              indexer.setGoal(IndexerGoal.STOP);
            })
        .withName(name);
  }

  /** Shoot at the hub while letting the driver keep translational control. */
  public static Command shoot(
      RobotContainer c, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return shootAtTarget(
        c, CommandFactory::hubTarget, xSupplier, ySupplier, "Superstructure.Shoot");
  }

  /**
   * Fixed shooting fallback used when vision is unavailable/unreliable. No aiming or distance
   * lookup: the arm holds its fixed shooting angle and the flywheel spins to its fixed shoot
   * velocity ({@link FlywheelGoal#SHOOT}). The ball is fed once both the arm and flywheel report
   * being at their goals. The driver retains full manual drive control (this command does not touch
   * the drivetrain, so the default joystick drive stays active).
   *
   * @param c robot container
   */
  public static Command shootFixed(RobotContainer c) {
    Arm arm = c.arm;
    Flywheel flywheel = c.flywheel;
    Indexer indexer = c.indexer;

    return Commands.parallel(
            arm.setGoalCommand(ArmGoal.SHOOTING),
            flywheel.setGoalCommand(FlywheelGoal.SHOOT),
            Commands.run(
                () -> {
                  boolean ready = arm.isAtGoal() && flywheel.atGoal();
                  Logger.recordOutput("CommandFactory/fixedReadyToShoot", ready);
                  indexer.setGoal(ready ? IndexerGoal.SHOOT : IndexerGoal.SLOWFORWARD);
                },
                indexer))
        .finallyDo(
            () -> {
              arm.setGoal(ArmGoal.STOW);
              flywheel.setGoal(FlywheelGoal.IDLE);
              indexer.setGoal(IndexerGoal.STOP);
            })
        .withName("Superstructure.ShootFixed");
  }

  public static Command shootTest(RobotContainer c) {
    Arm arm = c.arm;
    Flywheel flywheel = c.flywheel;
    Indexer indexer = c.indexer;

    return Commands.parallel(
            arm.setGoalCommand(ArmGoal.TEST),
            flywheel.setGoalCommand(FlywheelGoal.TEST),
            Commands.run(
                () -> {
                  boolean ready = arm.isAtGoal() && flywheel.atGoal();
                  Logger.recordOutput("CommandFactory/testReadyToShoot", ready);
                  indexer.setGoal(ready ? IndexerGoal.SHOOT : IndexerGoal.SLOWFORWARD);
                },
                indexer))
        .finallyDo(
            () -> {
              arm.setGoal(ArmGoal.STOW);
              flywheel.setGoal(FlywheelGoal.IDLE);
              indexer.setGoal(IndexerGoal.STOP);
            })
        .withName("Superstructure.ShootFixed");
  }

  public static Command passing(RobotContainer c) {
    Arm arm = c.arm;
    Flywheel flywheel = c.flywheel;
    Indexer indexer = c.indexer;

    return Commands.parallel(
            arm.setGoalCommand(ArmGoal.PASSING),
            flywheel.setGoalCommand(FlywheelGoal.SHOOT),
            Commands.run(
                () -> {
                  boolean ready = arm.isAtGoal() && flywheel.atGoal();
                  Logger.recordOutput("CommandFactory/testReadyToShoot", ready);
                  indexer.setGoal(ready ? IndexerGoal.SHOOT : IndexerGoal.SLOWFORWARD);
                },
                indexer))
        .finallyDo(
            () -> {
              arm.setGoal(ArmGoal.STOW);
              flywheel.setGoal(FlywheelGoal.IDLE);
              indexer.setGoal(IndexerGoal.STOP);
            })
        .withName("Superstructure.ShootFixed");
  }
}
