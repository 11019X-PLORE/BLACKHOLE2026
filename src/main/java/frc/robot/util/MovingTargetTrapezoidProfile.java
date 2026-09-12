package frc.robot.util;

import edu.wpi.first.math.MathUtil;

/**
 * A trapezoidal motion profile for a goal with constant velocity during the profile step.
 *
 * <p>The calculation is performed in the goal's moving frame, then transformed back to the
 * mechanism frame. This lets the profile use the distance to where the goal will be, not only where
 * it is now.
 */
public class MovingTargetTrapezoidProfile {
  private static final double EPSILON = 1e-9;

  private final Constraints constraints;

  public record Constraints(double maxVelocity, double maxAcceleration) {
    public Constraints {
      if (maxVelocity < 0.0 || maxAcceleration < 0.0) {
        throw new IllegalArgumentException("Constraints must be non-negative");
      }
    }
  }

  public record State(double position, double velocity) {}

  public MovingTargetTrapezoidProfile(Constraints constraints) {
    this.constraints = constraints;
  }

  public State calculate(double t, State current, State goal) {
    double dt = Math.max(0.0, t);
    if (dt <= EPSILON) {
      return current;
    }

    double maxVelocity = constraints.maxVelocity();
    double maxAcceleration = constraints.maxAcceleration();
    if (maxVelocity <= EPSILON) {
      return new State(current.position(), 0.0);
    }

    int direction = current.position() > goal.position() ? -1 : 1;
    double currentPosition = direction * current.position();
    double currentVelocity =
        MathUtil.clamp(direction * current.velocity(), -maxVelocity, maxVelocity);
    double goalPosition = direction * goal.position();
    double goalVelocity = MathUtil.clamp(direction * goal.velocity(), -maxVelocity, maxVelocity);

    double minRelativeVelocity = -maxVelocity - goalVelocity;
    double maxRelativeVelocity = maxVelocity - goalVelocity;
    double currentRelativeVelocity =
        MathUtil.clamp(currentVelocity - goalVelocity, minRelativeVelocity, maxRelativeVelocity);

    if (maxAcceleration <= EPSILON) {
      return transformFromMovingFrame(
          direction,
          dt,
          currentPosition + currentRelativeVelocity * dt,
          currentRelativeVelocity,
          goalVelocity);
    }

    double distanceToGoal = Math.max(0.0, goalPosition - currentPosition);
    if (currentRelativeVelocity > 0.0) {
      double stoppingDistance =
          currentRelativeVelocity * currentRelativeVelocity / (2.0 * maxAcceleration);
      if (stoppingDistance > distanceToGoal + EPSILON) {
        double acceleration =
            MathUtil.clamp(-currentRelativeVelocity / dt, -maxAcceleration, maxAcceleration);
        return integrateMovingFrame(
            direction,
            dt,
            currentPosition,
            currentRelativeVelocity,
            acceleration,
            minRelativeVelocity,
            maxRelativeVelocity,
            goalVelocity);
      }
    }

    if (maxRelativeVelocity <= EPSILON) {
      double acceleration =
          MathUtil.clamp(-currentRelativeVelocity / dt, -maxAcceleration, maxAcceleration);
      return integrateMovingFrame(
          direction,
          dt,
          currentPosition,
          currentRelativeVelocity,
          acceleration,
          minRelativeVelocity,
          maxRelativeVelocity,
          goalVelocity);
    }

    State movingFrameResult =
        calculateStationaryGoal(
            dt,
            currentPosition,
            currentRelativeVelocity,
            goalPosition,
            maxRelativeVelocity,
            maxAcceleration);
    return transformFromMovingFrame(
        direction, dt, movingFrameResult.position(), movingFrameResult.velocity(), goalVelocity);
  }

  private static State calculateStationaryGoal(
      double t,
      double currentPosition,
      double currentVelocity,
      double goalPosition,
      double maxVelocity,
      double maxAcceleration) {
    // Standard truncated-trapezoid math, applied in the moving frame.
    double cutoffBegin = currentVelocity / maxAcceleration;
    double cutoffDistBegin = cutoffBegin * cutoffBegin * maxAcceleration / 2.0;

    double fullTrapezoidDist = cutoffDistBegin + (goalPosition - currentPosition);
    double accelerationTime = maxVelocity / maxAcceleration;

    double fullSpeedDist =
        fullTrapezoidDist - accelerationTime * accelerationTime * maxAcceleration;

    if (fullSpeedDist < 0.0) {
      accelerationTime = Math.sqrt(Math.max(0.0, fullTrapezoidDist / maxAcceleration));
      fullSpeedDist = 0.0;
    }

    double endAccel = accelerationTime - cutoffBegin;
    double endFullSpeed = endAccel + fullSpeedDist / maxVelocity;
    double endDecel = endFullSpeed + accelerationTime;

    if (t < endAccel) {
      return new State(
          currentPosition + (currentVelocity + t * maxAcceleration / 2.0) * t,
          currentVelocity + t * maxAcceleration);
    } else if (t < endFullSpeed) {
      double position =
          currentPosition
              + (currentVelocity + endAccel * maxAcceleration / 2.0) * endAccel
              + maxVelocity * (t - endAccel);
      return new State(position, maxVelocity);
    } else if (t <= endDecel) {
      double timeLeft = endDecel - t;
      return new State(
          goalPosition - (timeLeft * maxAcceleration / 2.0) * timeLeft, timeLeft * maxAcceleration);
    } else {
      return new State(goalPosition, 0.0);
    }
  }

  private static State integrateMovingFrame(
      int direction,
      double t,
      double relativePosition,
      double relativeVelocity,
      double relativeAcceleration,
      double minRelativeVelocity,
      double maxRelativeVelocity,
      double goalVelocity) {
    double position = relativePosition;
    double velocity = MathUtil.clamp(relativeVelocity, minRelativeVelocity, maxRelativeVelocity);

    if (Math.abs(relativeAcceleration) <= EPSILON) {
      position += velocity * t;
    } else {
      double velocityLimit = relativeAcceleration > 0.0 ? maxRelativeVelocity : minRelativeVelocity;
      double timeToVelocityLimit = (velocityLimit - velocity) / relativeAcceleration;

      if (timeToVelocityLimit > EPSILON && timeToVelocityLimit < t) {
        position +=
            velocity * timeToVelocityLimit
                + 0.5 * relativeAcceleration * timeToVelocityLimit * timeToVelocityLimit;
        velocity = velocityLimit;
        position += velocity * (t - timeToVelocityLimit);
      } else if (timeToVelocityLimit <= EPSILON
          && ((relativeAcceleration > 0.0 && velocity >= velocityLimit)
              || (relativeAcceleration < 0.0 && velocity <= velocityLimit))) {
        velocity = velocityLimit;
        position += velocity * t;
      } else {
        position += velocity * t + 0.5 * relativeAcceleration * t * t;
        velocity =
            MathUtil.clamp(
                velocity + relativeAcceleration * t, minRelativeVelocity, maxRelativeVelocity);
      }
    }

    return transformFromMovingFrame(direction, t, position, velocity, goalVelocity);
  }

  private static State transformFromMovingFrame(
      int direction, double t, double position, double velocity, double goalVelocity) {
    return new State(
        direction * (position + goalVelocity * t), direction * (velocity + goalVelocity));
  }
}
