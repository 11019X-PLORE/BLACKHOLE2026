// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.Geoffrey;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
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
      TrajectoryConfig trajConfig) {

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

    double[] ballistics = TrajectoryCalculator.solve(dx, hMax, trajConfig);
    double vH_req = ballistics[0];
    double vZ_req = ballistics[1];

    // 3. Launch Vector (V_l = V_ball - V_muzzle)
    Translation3d vBallField =
        new Translation3d(vH_req * angleToTarget.getCos(), vH_req * angleToTarget.getSin(), vZ_req);
    Translation3d vLaunch = vBallField.minus(vMuzzle);

    // 4. Calculate Derivatives for aLaunch
    Translation2d vRel = vMuzzle.toTranslation2d().times(-1.0);
    double d_dx = (deltaPos.getX() * vRel.getX() + deltaPos.getY() * vRel.getY()) / dx;
    double d_theta_dt = (deltaPos.getX() * vRel.getY() - deltaPos.getY() * vRel.getX()) / (dx * dx);

    // Calculate d[vx, vy]/dx
    double[] ballisticsDeriv = TrajectoryCalculator.solveDerivative(dx, hMax, trajConfig);

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
    double z = vLaunch.getZ();
    double vz = aLaunch.getZ();
    double h = vLaunch.toTranslation2d().getNorm(); // Horizontal magnitude of vLaunch
    double vh = (x * vx + y * vy) / h; // d/dt horizontal magnitude
    double denH = h * h + z * z;

    // Position & Velocity
    double hoodPos = Math.atan2(z, h);
    double hoodVel = (h * vz - z * vh) / denH;

    // Acceleration (Derivative of hoodVel)
    // Again, assuming z_double_dot and h_double_dot are 0 (no Jerk)
    double alphaHood =
        ((0 - 0) * denH - (h * vz - z * vh) * (2 * h * vh + 2 * z * vz)) / (denH * denH);
    double hoodAccel = alphaHood; // Hood is relative to the turret plate, usually

    // 7. SHOOTER FF
    double shooterVel = vLaunch.getNorm();
    // Acceleration of the flywheel magnitude: d/dt sqrt(x^2 + y^2 + z^2)
    double shooterAccel = (x * vx + y * vy + z * vz) / shooterVel;

    return new ShooterSetpoint(
        shooterVel, shooterAccel, turretPos, turretVel, turretAccel, hoodPos, hoodVel, hoodAccel);
  }
}
