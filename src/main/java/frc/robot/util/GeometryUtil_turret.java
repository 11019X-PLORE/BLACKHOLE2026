package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

public class GeometryUtil_turret {

  /** Wrap 到 [-π, π] */
  public static double wrapRadians(double radians) {
    return MathUtil.inputModulus(radians, -Math.PI, Math.PI);
  }

  /** Wrap Rotation2d 到 [-π, π] */
  public static Rotation2d wrapRotation(Rotation2d rot) {
    return Rotation2d.fromRadians(wrapRadians(rot.getRadians()));
  }

  /** 限制在范围内 */
  public static double clamp(double val, double min, double max) {
    return MathUtil.clamp(val, min, max);
  }

  /** 同时 wrap + clamp */
  public static double wrapAndClamp(double radians, double min, double max) {
    double wrapped = wrapRadians(radians);
    return clamp(wrapped, min, max);
  }
}
