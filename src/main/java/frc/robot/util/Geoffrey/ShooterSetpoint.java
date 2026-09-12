// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.Geoffrey;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.Logger;

/**
 * A complete, immutable description of everything the robot needs to make a shot at a fixed field
 * target.
 *
 * <p>This offseason robot has <b>no turret and a fixed shooting arm angle</b>. Aiming is therefore
 * only about the <b>chassis yaw</b> (rotated by the swerve) and the <b>flywheel velocity</b>. Both
 * are looked up purely as a function of the horizontal distance to the hub, using two functions fit
 * from real robot data ({@link #shooterVelocityFromDistance} and {@link
 * #timeOfFlightFromDistance}).
 *
 * <p><b>Shoot on the move.</b> Because the ball inherits the robot's velocity at launch, a moving
 * robot must aim "upstream". We solve this with a fixed-point iteration: estimate the time of
 * flight, shift the aim point by {@code robotVelocity * timeOfFlight}, recompute the distance/time,
 * and repeat until it converges.
 *
 * <p><b>Turning feedforward.</b> The yaw-rate feedforward is derived from the target's velocity
 * <i>relative to the robot</i> (for a stationary hub, that is simply {@code -robotVelocity})
 * projected perpendicular to the line of sight.
 *
 * @param shooterVelocityRadsPerSec required flywheel velocity (rad/s), from real data
 * @param yawPositionRadians field-relative heading the chassis should hold (rad)
 * @param yawVelocityRadsPerSec feedforward yaw rate to keep tracking while translating (rad/s)
 * @param distanceMeters horizontal distance from the shooter to the (virtual) aim point (m)
 * @param timeOfFlightSeconds estimated ball time of flight for this shot (s)
 * @param valid whether a usable shot was found
 */
public record ShooterSetpoint(
    double shooterVelocityRadsPerSec,
    double pitchAngleRads,
    double yawPositionRadians,
    double yawVelocityRadsPerSec,
    double distanceMeters,
    double timeOfFlightSeconds,
    boolean valid) {

  /** Number of fixed-point iterations used to converge the shoot-on-the-move aim point. */
  private static final int kMovingShotIterations = 5;

  private static final double[] distances = {2.43, 2.8, 3.4, 3.658, 4.021};
  private static final double[] speeds = {-270, -280, -300, -300, -350};
  private static final double[] angleDegs = {75, 75, 75, 70, 70};

  /** An empty, invalid setpoint. Handy as a default before the first solve. */
  public static final ShooterSetpoint kEmpty = new ShooterSetpoint(0, 0, 0, 0, 0, 0, false);

  /** Convenience accessor for the yaw as a {@link Rotation2d}. */
  public Rotation2d yaw() {
    return new Rotation2d(yawPositionRadians);
  }

  // ==================== REAL-DATA LOOKUPS (fill these in) ====================

  /**
   * Required flywheel velocity (rad/s) to make the shot at a given horizontal hub distance.
   *
   * <p>TODO: fit this from real robot data (distance in metres -> flywheel velocity in rad/s).
   *
   * @param distanceMeters horizontal distance to the hub (m)
   * @return flywheel velocity setpoint (rad/s)
   */
  public static double shooterVelocityFromDistance(double distanceMeters) {
    // TODO: fill in with the empirical distance -> flywheel velocity fit.
    // return -300;
    if (distanceMeters <= distances[0]) return speeds[0];
    if (distanceMeters >= distances[distances.length - 1]) return speeds[speeds.length - 1];
    for (int i = 0; i < distances.length - 1; i++) {
      if (distanceMeters >= distances[i] && distanceMeters < distances[i + 1]) {
        return speeds[i]
            + (speeds[i + 1] - speeds[i])
                * (distanceMeters - distances[i])
                / (distances[i + 1] - distances[i]);
      }
    }
    return speeds[speeds.length - 1];
  }

  public static double shooterPitchFromDistance(double distanceMeters) {
    // double angle = 0;
    // if (distanceMeters > 4.073) {
    //   angle = 70;
    // } else if (distanceMeters > 3.5) {
    //   angle = 105.541 - (8.726 * distanceMeters);
    // } else {
    //   angle = 75;
    // }
    // return Math.toRadians(angle);

    if (distanceMeters <= distances[0]) return angleDegs[0];
    if (distanceMeters >= distances[distances.length - 1]) return angleDegs[angleDegs.length - 1];
    for (int i = 0; i < distances.length - 1; i++) {
      if (distanceMeters >= distances[i] && distanceMeters < distances[i + 1]) {
        return angleDegs[i]
            + (angleDegs[i + 1] - angleDegs[i])
                * (distanceMeters - distances[i])
                / (distances[i + 1] - distances[i]);
      }
    }
    return angleDegs[angleDegs.length - 1];
  }

  /**
   * Ball time of flight (s) for a shot at a given horizontal hub distance.
   *
   * <p>TODO: fit this from real robot data (distance in metres -> time of flight in seconds).
   *
   * @param distanceMeters horizontal distance to the hub (m)
   * @return time of flight (s)
   */
  public static double timeOfFlightFromDistance(double distanceMeters) {
    // TODO: fill in with the empirical distance -> time-of-flight fit.
    return 1;
  }

  // ==================== SETPOINT BUILDER ====================

  /**
   * Build a full shooting setpoint aimed at a fixed field target, compensating for robot motion.
   *
   * @param base the Geoffrey joint the shot is referenced from (normally the drive/chassis). Its
   *     global pose gives the shooter position and its global velocity gives the robot's
   *     field-relative motion used for the shoot-on-the-move compensation and yaw feedforward.
   * @param target the field-relative point to shoot at (alliance flipping already applied).
   * @return a populated {@link ShooterSetpoint}.
   */
  public static ShooterSetpoint makeSetpoint(PhysicalJoint base, Translation2d target) {
    Translation2d robot = base.getGlobalPose().getTranslation().toTranslation2d();
    SimpleMatrix v = base.getGlobalVelocity();
    Translation2d robotVelocity = new Translation2d(v.get(0), v.get(1));

    // --- Shoot-on-the-move: iterate to find the virtual aim point ---
    // The ball inherits the robot's velocity at launch, so a moving robot must aim upstream. Shift
    // the aim point by robotVelocity * timeOfFlight; since the time of flight depends on the
    // distance (which depends on the aim point) we converge it with a fixed-point iteration.
    Translation2d aimPoint = target;
    double timeOfFlight = 0.0;
    for (int i = 0; i < kMovingShotIterations; i++) {
      double d = robot.getDistance(aimPoint);
      timeOfFlight = timeOfFlightFromDistance(d);
      aimPoint = target.minus(robotVelocity.times(timeOfFlight));
    }

    double dx = aimPoint.getX() - robot.getX();
    double dy = aimPoint.getY() - robot.getY();
    double distance = Math.hypot(dx, dy);
    double yaw = Math.atan2(dy, dx);

    // Guard against a degenerate aim point right on top of the robot.
    if (distance < 1e-4) {
      return new ShooterSetpoint(0, 0, yaw, 0, distance, timeOfFlight, false);
    }

    double speed = shooterVelocityFromDistance(distance);

    double pitch = Math.toRadians(shooterPitchFromDistance(distance));
    Logger.recordOutput("ShooterSetpoint", Math.toDegrees(pitch));

    // --- Turning feedforward ---
    // The aiming bearing changes at a rate set by the target's velocity *relative to the robot*.
    // For a stationary hub that relative velocity is just -robotVelocity. Projecting it onto the
    // direction perpendicular to the line of sight and dividing by distance gives the yaw rate:
    //   d(yaw)/dt = (dx * relVy - dy * relVx) / distance^2
    double relVx = -robotVelocity.getX();
    double relVy = -robotVelocity.getY();
    double yawVelocity = (dx * relVy - dy * relVx) / (distance * distance);

    boolean valid = speed > 1e-6;
    return new ShooterSetpoint(speed, pitch, yaw, yawVelocity, distance, timeOfFlight, valid);
  }
}
