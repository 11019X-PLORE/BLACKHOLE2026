package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Ports;

public class ArmConstants {
  public static final int kArmId = Ports.kArm;
  public static final boolean kArmInverted = true;

  public static final double kArmGearRatio = 5.0 * 5.0 * ((65.0 / 10.0) + 1);

  // --- Control gains ---
  public static final double kP = 1000.0;
  public static final double kI = 0.0;
  public static final double kD = 30.0;
  public static final double kS = 0.0;
  public static final double kV = 0.0;
  public static final double kA = 0.0;
  public static final double kG = 0.0;

  public static final double kToleranceRads = Units.degreesToRadians(5.0);

  // --- Motion limits ---
  public static final double kMaxVelocityRadPerSec = Units.degreesToRadians(180.0);
  public static final double kMaxAccelerationRadPerSecSq = Units.degreesToRadians(360.0);

  public static final double kMaxVoltage = 12;

  // --- Setpoints (angle measured from horizontal) ---
  public static final double kMinAngle = Units.degreesToRadians(60.0 - 82.16);
  public static final double kMaxAngle = Units.degreesToRadians(60.0 + 44.75);
  public static final double kInitialAngle = Units.degreesToRadians(60.0 + 44.75);
  public static final double kIntakeAngle =
      Units.degreesToRadians(60.0 - 82.16); // deployed to floor
  public static final double kStowAngle = Units.degreesToRadians(60.0); // folded up
  public static final double kShootAngle = Units.degreesToRadians(70.0); // default shooting angle
  public static final double kPassingAngle = Units.degreesToRadians(62.5); // default shooting angle

  public static final double backlashAngle = Units.degreesToRadians(5); // default shooting angle

  // Test/tunable default
  public static final double kFixAngle = Units.degreesToRadians(45.0);

  // Feedforward physics
  public static final double kInertia = 0.125; // TODO
  public static final double kT = 0.01537; // torque constant

  /**
   * Static transform from the chassis (drive) origin to the arm pivot. Used by the Geoffrey
   * physical joint chain to locate the arm on the robot.
   */
  public static final Transform3d kChassisToArmPivot =
      new Transform3d(new Translation3d(0.20, 0.0, 0.25), new Rotation3d(0.0, 0.0, 0.0));
}
