package frc.robot.util.Geoffrey;

public class TrajectoryCalculator {

  /**
   * Compute launch velocities (vx, vy) for a given horizontal distance and desired apex height.
   *
   * @param dx horizontal distance to target (m)
   * @param hmax desired apex height (m), must be greater than DY
   * @return double[]{vx, vy} in m/s
   */
  public static double[] solve(double dx, double hmax, TrajectoryConfig config) {
    // 1. Vacuum baseline from kinematics
    double vyVac = Math.sqrt(2.0 * config.G * hmax);
    double vxVac;

    if (Math.abs(config.DY) < 1e-10) {
      vxVac = config.G * dx / (2.0 * vyVac);
    } else {
      double disc = vyVac * vyVac - 2.0 * config.G * config.DY;
      if (disc < 0) {
        return new double[] {0, 0}; // impossible trajectory
      }
      double sqrtDisc = Math.sqrt(disc);
      if (config.APEX_BEFORE_TARGET) {
        vxVac = dx * (vyVac - sqrtDisc) / (2.0 * config.DY);
      } else {
        vxVac = dx * (vyVac + sqrtDisc) / (2.0 * config.DY);
      }
    }

    // 2. Polynomial aero correction: degree-3 features
    double dx2 = dx * dx;
    double dx3 = dx2 * dx;
    double h2 = hmax * hmax;
    double h3 = h2 * hmax;
    double[] features = {1, dx, hmax, dx2, dx * hmax, h2, dx3, dx2 * hmax, dx * h2, h3};

    double dvx = dot(config.DVX_COEFFS, features);
    double dvy = dot(config.DVY_COEFFS, features);

    // 3. Final velocity = vacuum + aero correction
    return new double[] {vxVac + dvx, vyVac + dvy};
    // return new double[] {vxVac, vyVac};
  }

  private static double dot(double[] a, double[] b) {
    double sum = 0;
    for (int i = 0; i < a.length; i++) {
      sum += a[i] * b[i];
    }
    return sum;
  }

  public static double[] solveDerivative(double dx, double hmax, TrajectoryConfig config) {
    // --- 1. Derivative of Vacuum Components ---
    // vyVac = sqrt(2 * G * hmax). Since hmax is constant w.r.t dx, d(vyVac)/dx = 0
    double d_vyVac_dx = 0;

    double d_vxVac_dx;
    double vyVac = Math.sqrt(2.0 * config.G * hmax);

    if (Math.abs(config.DY) < 1e-10) {
      // vxVac = (G / 2*vyVac) * dx
      d_vxVac_dx = config.G / (2.0 * vyVac);
    } else {
      double disc = vyVac * vyVac - 2.0 * config.G * config.DY;
      if (disc < 0) return new double[] {0, 0};

      double sqrtDisc = Math.sqrt(disc);
      // vxVac is essentially (Constant * dx), so the derivative is just the constant
      if (config.APEX_BEFORE_TARGET) {
        d_vxVac_dx = (vyVac - sqrtDisc) / (2.0 * config.DY);
      } else {
        d_vxVac_dx = (vyVac + sqrtDisc) / (2.0 * config.DY);
      }
    }

    // --- 2. Derivative of Aero Correction ---
    // Original features: {1, dx, hmax, dx2, dx*hmax, h2, dx3, dx2*hmax, dx*h2, h3}
    // We derive each feature with respect to dx:
    double[] dFeatures = new double[10];
    dFeatures[0] = 0; // d(1)/dx
    dFeatures[1] = 1; // d(dx)/dx
    dFeatures[2] = 0; // d(hmax)/dx
    dFeatures[3] = 2.0 * dx; // d(dx^2)/dx
    dFeatures[4] = hmax; // d(dx * hmax)/dx
    dFeatures[5] = 0; // d(hmax^2)/dx
    dFeatures[6] = 3.0 * dx * dx; // d(dx^3)/dx
    dFeatures[7] = 2.0 * dx * hmax; // d(dx^2 * hmax)/dx
    dFeatures[8] = hmax * hmax; // d(dx * hmax^2)/dx
    dFeatures[9] = 0; // d(hmax^3)/dx

    double d_dvx_dx = dot(config.DVX_COEFFS, dFeatures);
    double d_dvy_dx = dot(config.DVY_COEFFS, dFeatures);

    // --- 3. Final Summation ---
    // d(Total)/dx = d(Vacuum)/dx + d(Aero)/dx
    return new double[] {d_vxVac_dx + d_dvx_dx, d_vyVac_dx + d_dvy_dx};
  }

  /**
   * Compute the required flywheel surface speed for a given total ball speed.
   *
   * @param v total ball speed (m/s)
   * @return flywheel surface speed (m/s)
   */
  public static double getFlywheelSetpoint(double v) {
    // if (v > 7.3) {
    //   return 4.74446 * Math.pow(1.17377, v);
    // }
    // return 15;
    // return 0.85 * 4.40309 * Math.pow(1.17819, v);
    // return 4.62907 * Math.pow(1.18664, v); // placeholder for testing
    // return 3.44691 * v - 9.06894;

    return 1.1 * 4.44865 * Math.pow(1.14391, v);
  }

  public static double getHoodSetpoint(double a) {
    // return Math.toRadians((0.746204 * Math.toDegrees(a)) + 14.48929);
    return Math.toRadians((0.901026 * Math.toDegrees(a)) + 5.23916);

    // return Math.toRadians((0.0774547 * Math.pow(a, 1.5867)));
    // return a;
  }

  public static double getFlywheelAcceleration(double v, double a) {
    double dx = 1e-6; // small delta for numerical differentiation
    double dv = getFlywheelSetpoint(v + dx) - getFlywheelSetpoint(v);
    return (dv / dx) * a;
  }
}
