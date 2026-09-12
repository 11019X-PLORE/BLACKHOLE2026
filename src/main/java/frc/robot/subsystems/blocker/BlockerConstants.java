package frc.robot.subsystems.blocker;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Ports;

public class BlockerConstants {
  public static final int kBlockerId = Ports.kBlocker;
  public static final boolean kBlockerInverted = true;

  public static final double kBlockerGearRatio = 5.0 * 5.0 * 40.0 / 24.0;

  // --- Control gains ---
  public static final double kP = 100.0;
  public static final double kI = 0.0;
  public static final double kD = 10.0;
  public static final double kS = 0.5;
  public static final double kV = 0.0;
  public static final double kA = 0.0;
  public static final double kG = 0.0;

  public static final double kToleranceRads = Units.degreesToRadians(2.0);

  // --- Motion limits ---
  public static final double kMaxVelocityRadPerSec = Units.degreesToRadians(180.0);
  public static final double kMaxAccelerationRadPerSecSq = Units.degreesToRadians(360.0);

  // --- Setpoints (angle measured from the retracted/open position) ---
  public static final double kMinAngle = Units.degreesToRadians(0.0);
  public static final double kMaxAngle = Units.degreesToRadians(140.0);
  public static final double kInitialAngle = Units.degreesToRadians(-10.0);
  public static final double kAvoidCollisionAngle = Units.degreesToRadians(30.0);
  public static final double kOpenAngle = Units.degreesToRadians(80.0); // retracted, out of the way
  public static final double kHalfOpenAngle =
      Units.degreesToRadians(40.0); // retracted, out of the way

  public static final double kBlockingAngle = Units.degreesToRadians(150.0); // deployed to block

  // Test/tunable default
  public static final double kFixAngle = Units.degreesToRadians(45.0);

  public static final double adjustZeroAngleRads = Units.degreesToRadians(3.0);

  // Feedforward physics
  public static final double kInertia = 0.05; // TODO
  public static final double kT = 0.01537; // torque constant

  /**
   * Static transform from the chassis (drive) origin to the blocker pivot. Used by the Geoffrey
   * physical joint chain to locate the blocker on the robot.
   */
  public static final Transform3d kChassisToBlockerPivot =
      new Transform3d(new Translation3d(-0.20, 0.0, 0.30), new Rotation3d(0.0, 0.0, 0.0));
}
