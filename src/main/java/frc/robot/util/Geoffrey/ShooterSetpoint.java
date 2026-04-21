// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.Geoffrey;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import org.ejml.simple.SimpleMatrix;

/** shout out to 254 */
public class ShooterSetpoint {
  public double shooterVelocityMetersPerSec;
  public double shooterAccelerationMetersPerSecSquared;
  public double turretPositionRadians;
  public double turretVelocityRadsPerSec;
  public double turretAccelerationRadsPerSecSquared;
  public double hoodPositionRadians;
  public double hoodVelocityRadsPerSec;
  public double hoodAccelerationRadsPerSecSquared;

  public ShooterSetpoint(
      double shooterVelocityMetersPerSec,
      double shooterAccelerationMetersPerSecSquared,
      double turretPositionRadians,
      double turretVelocityRadsPerSec,
      double turretAccelerationRadsPerSecSquared,
      double hoodPositionRadians,
      double hoodVelocityRadsPerSec,
      double hoodAccelerationRadsPerSecSquared) {
    this.shooterVelocityMetersPerSec = shooterVelocityMetersPerSec;
    this.shooterAccelerationMetersPerSecSquared = shooterAccelerationMetersPerSecSquared;
    this.turretPositionRadians = turretPositionRadians;
    this.turretVelocityRadsPerSec = turretVelocityRadsPerSec;
    this.turretAccelerationRadsPerSecSquared = turretAccelerationRadsPerSecSquared;
    this.hoodPositionRadians = hoodPositionRadians;
    this.hoodVelocityRadsPerSec = hoodVelocityRadsPerSec;
    this.hoodAccelerationRadsPerSecSquared = hoodAccelerationRadsPerSecSquared;
  }

  public static ShooterSetpoint makeSetpoint(
      PhysicalJoint muzzleJoint,
      Translation2d targetPos2d,
      double hMax,
      double minHoodAngleRads,
      double maxHoodAngleRads,
      TrajectoryConfig trajConfig,
      boolean use3dRotation) {

    // 1. Get current Muzzle State (Pose, Vel, Accel)
    Translation3d muzzlePos = muzzleJoint.getGlobalPose().getTranslation();
    SimpleMatrix muzzleV6 = muzzleJoint.getGlobalVelocity();
    SimpleMatrix muzzleA6 = muzzleJoint.getGlobalAcceleration();

    Translation3d vMuzzle = new Translation3d(muzzleV6.get(0), muzzleV6.get(1), muzzleV6.get(2));
    Translation3d aMuzzle = new Translation3d(muzzleA6.get(0), muzzleA6.get(1), muzzleA6.get(2));
    double robotOmega = muzzleV6.get(5);
    double robotAlpha = muzzleA6.get(5);

    // 2. Geometry & Ballistics
    Translation2d deltaPos = targetPos2d.minus(muzzlePos.toTranslation2d());
    double dx = deltaPos.getNorm();
    Rotation2d angleToTarget = deltaPos.getAngle();

    double validHmax = hMax;
    double tempHmax = hMax;

    Translation3d vLaunch = new Translation3d();
    double vH_req = 0;
    double vZ_req = 0;
    double z = 0;
    double h = 0;
    double hoodPos = 0;
    Translation3d vBallField = new Translation3d();
    double high = trajConfig.max_hMax;
    double low = trajConfig.min_hMax;

    for (int i = 0; i < 10; i++) {
      double[] ballistics = TrajectoryCalculator.solve(dx, validHmax, trajConfig);
      vH_req = ballistics[0];
      vZ_req = ballistics[1];

      // 3. Launch Vector (V_l = V_ball - V_muzzle)
      vBallField =
          new Translation3d(
              vH_req * angleToTarget.getCos(), vH_req * angleToTarget.getSin(), vZ_req);
      vLaunch = vBallField.minus(vMuzzle);

      z = vLaunch.getZ();
      h = vLaunch.toTranslation2d().getNorm(); // Horizontal magnitude of vLaunch
      hoodPos = Math.atan2(z, h);

      if (i == 0) {
        if (hoodPos >= minHoodAngleRads && hoodPos <= maxHoodAngleRads) {
          break; // If the initial solution is valid, no need to iterate
        }
        if (hoodPos < minHoodAngleRads) {
          low = tempHmax;
        } else {
          high = tempHmax;
        }
      } else {
        if (tempHmax > hMax) {
          if (hoodPos < minHoodAngleRads) {
            low = tempHmax;
          } else {
            high = tempHmax;
            validHmax = tempHmax;
          }
        } else {
          if (hoodPos < maxHoodAngleRads) {
            low = tempHmax;
            validHmax = tempHmax;
          } else {
            high = tempHmax;
          }
        }
      }
      tempHmax = (high + low) / 2.0;
    }

    // 4. Calculate Derivatives for aLaunch
    Translation2d vRel = vMuzzle.toTranslation2d().times(-1.0);
    double d_dx = (deltaPos.getX() * vRel.getX() + deltaPos.getY() * vRel.getY()) / dx;
    double d_theta_dt = (deltaPos.getX() * vRel.getY() - deltaPos.getY() * vRel.getX()) / (dx * dx);

    // Calculate d[vx, vy]/dx
    double[] ballisticsDeriv = TrajectoryCalculator.solveDerivative(dx, validHmax, trajConfig);

    double dvH_dt = ballisticsDeriv[0] * d_dx;
    double dvZ_dt = ballisticsDeriv[1] * d_dx;

    // aBallField is the derivative of the required field-space ball velocity
    Translation3d aBallField =
        new Translation3d(
            dvH_dt * angleToTarget.getCos() - vH_req * angleToTarget.getSin() * d_theta_dt,
            dvH_dt * angleToTarget.getSin() + vH_req * angleToTarget.getCos() * d_theta_dt,
            dvZ_dt);
    // aLaunch = d/dt vLaunch = aBallField - aMuzzle
    Translation3d aLaunch = aBallField.minus(aMuzzle);

    // 5. TURRET FF (Azimuth)
    double x = vLaunch.getX();
    double y = vLaunch.getY();
    double vx = aLaunch.getX();
    double vy = aLaunch.getY();
    double denT = x * x + y * y;

    // Position & Velocity
    double turretPos =
        MathUtil.angleModulus(
            vLaunch
                .toTranslation2d()
                .getAngle()
                .minus(muzzleJoint.getGlobalPose().getRotation().toRotation2d())
                .getRadians());
    double omegaField = (x * vy - y * vx) / denT;
    double turretVel = omegaField - robotOmega;

    // Acceleration (Derivative of omegaField - robotAlpha)
    // Assume jerk (ax_dot, ay_dot) is 0, but the geometric acceleration is non-zero
    double alphaField =
        ((0 - 0) * denT - (x * vy - y * vx) * (2 * x * vx + 2 * y * vy)) / (denT * denT);
    double turretAccel = alphaField - robotAlpha;

    // 6. HOOD FF (Elevation)
    double vz = aLaunch.getZ();
    double vh = (x * vx + y * vy) / h; // d/dt horizontal magnitude
    double denH = h * h + z * z;

    // Position & Velocity
    double hoodVel = (h * vz - z * vh) / denH;

    // Acceleration (Derivative of hoodVel)
    // Assuming z_double_dot, x_double_dot, y_double_dot are 0 (no Jerk in field frame)
    // Note: h_double_dot (vh_dot) is NOT 0 due to centrifugal acceleration!
    double vh_dot = (vx * vx + vy * vy - vh * vh) / h;
    double u_prime = -z * vh_dot;
    double alphaHood =
        (u_prime * denH - (h * vz - z * vh) * (2 * h * vh + 2 * z * vz)) / (denH * denH);
    double hoodAccel = alphaHood; // Hood is relative to the turret plate, usually

    // 7. SHOOTER FF
    double shooterVel = vLaunch.getNorm();
    // Acceleration of the flywheel magnitude: d/dt sqrt(x^2 + y^2 + z^2)
    double shooterAccel = (x * vx + y * vy + z * vz) / shooterVel;

    // 8. (Optional) 3D Rotation Correction for tilted chassis
    // When use3dRotation=true, the full 3D orientation of the muzzle joint is used to remap
    // vLaunch into the robot-body frame, compensating for chassis pitch and roll.
    // Velocity/acceleration FFs are kept from the flat calculation — negligible error when
    // stationary.
    if (use3dRotation) {
      Rotation3d muzzleRot3d = muzzleJoint.getGlobalPose().getRotation();

      // Express vLaunch in the muzzle's local (robot-body) frame
      Translation3d vLaunchLocal = vLaunch.rotateBy(muzzleRot3d.unaryMinus());
      double xL = vLaunchLocal.getX();
      double yL = vLaunchLocal.getY();
      double zL = vLaunchLocal.getZ();
      double hL = Math.sqrt(xL * xL + yL * yL);

      // Recalculate position setpoints only
      turretPos = MathUtil.angleModulus(Math.atan2(yL, xL));
      hoodPos = Math.atan2(zL, hL);
      // turretVel, turretAccel, hoodVel, hoodAccel, shooterAccel are unchanged (robot is stationary
      // when tilted)
    }

    return new ShooterSetpoint(
        shooterVel, shooterAccel, turretPos, turretVel, turretAccel, hoodPos, hoodVel, hoodAccel);
  }

  /**
   * Creates a simple setpoint that locks the turret to point at the target (hub) with a fixed hood
   * angle and shooter speed. This is useful for vision tracking where the turret needs to
   * continuously face the hub so the camera can always see the AprilTags.
   *
   * @param muzzleJoint The physical joint representing the muzzle position
   * @param targetPos2d The 2D position of the target (hub) on the field
   * @param hoodAngleRads The fixed hood angle in radians
   * @param shooterSpeed The fixed shooter speed (0 if not shooting, just tracking)
   * @return A ShooterSetpoint with turret tracking and fixed hood/shooter values
   */
  public static ShooterSetpoint makeSetpoint(
      PhysicalJoint muzzleJoint,
      Translation2d targetPos2d,
      double hoodAngleRads,
      double shooterSpeed) {

    // 1. Get current Muzzle State (Pose, Vel, Accel)
    Translation3d muzzlePos = muzzleJoint.getGlobalPose().getTranslation();
    SimpleMatrix muzzleV6 = muzzleJoint.getGlobalVelocity();
    SimpleMatrix muzzleA6 = muzzleJoint.getGlobalAcceleration();

    Translation3d vMuzzle = new Translation3d(muzzleV6.get(0), muzzleV6.get(1), muzzleV6.get(2));
    double robotOmega = muzzleV6.get(5);
    double robotAlpha = muzzleA6.get(5);

    // 2. Calculate vector from muzzle to target
    Translation2d deltaPos = targetPos2d.minus(muzzlePos.toTranslation2d());
    Rotation2d angleToTarget = deltaPos.getAngle();

    // 3. Calculate turret position (angle to target relative to robot heading)
    double turretPos =
        MathUtil.angleModulus(
            angleToTarget
                .minus(muzzleJoint.getGlobalPose().getRotation().toRotation2d())
                .getRadians());

    // 4. Calculate turret velocity feedforward
    // d_theta_dt is the rate of change of the angle to target in field frame
    Translation2d vRel =
        vMuzzle.toTranslation2d().times(-1.0); // Relative velocity of target w.r.t. muzzle
    double dx = deltaPos.getNorm();
    double d_theta_dt = (deltaPos.getX() * vRel.getY() - deltaPos.getY() * vRel.getX()) / (dx * dx);
    double turretVel = d_theta_dt - robotOmega;

    // 5. Calculate turret acceleration feedforward
    // Using the derivative of the angular velocity
    double x = deltaPos.getX();
    double y = deltaPos.getY();
    double vx = vRel.getX();
    double vy = vRel.getY();
    double denT = x * x + y * y;

    // alphaField = d/dt(d_theta_dt), assuming no acceleration of muzzle in XY plane for simplicity
    double alphaField = -(x * vy - y * vx) * (2 * x * vx + 2 * y * vy) / (denT * denT);
    double turretAccel = alphaField - robotAlpha;

    // Return setpoint with turret tracking, fixed hood angle, and fixed shooter speed
    // Hood velocity and acceleration are 0 since we're holding a fixed angle
    // Shooter acceleration is 0 since we're holding a fixed speed
    return new ShooterSetpoint(
        shooterSpeed, 0, turretPos, turretVel, turretAccel, hoodAngleRads, 0, 0);
  }

  @Override
  public String toString() {
    return "ShooterSetpoint:{"
        + "shooterVelocityMetersPerSec = "
        + shooterVelocityMetersPerSec
        + "\n shooterAccelerationMetersPerSecSquared = "
        + shooterAccelerationMetersPerSecSquared
        + "\n turretPositionRadians = "
        + turretPositionRadians
        + "\n turretVelocityRadsPerSec = "
        + turretVelocityRadsPerSec
        + "\n turretAccelerationRadsPerSecSquared = "
        + turretAccelerationRadsPerSecSquared
        + "\n hoodPositionRadians = "
        + hoodPositionRadians
        + "\n hoodVelocityRadsPerSec = "
        + hoodVelocityRadsPerSec
        + "\n hoodAccelerationRadsPerSecSquared = "
        + hoodAccelerationRadsPerSecSquared
        + "}";
  }

  public static void main(String[] args) {
    ShooterSetpoint setpoint =
        makeSetpoint(
            PhysicalJoint.ground,
            new Translation2d(6, 0),
            2.2,
            Math.toRadians(13),
            Math.toRadians(38),
            TrajectoryConfig.getHubConfig(),
            false);
    System.out.println(setpoint);
  }
}
