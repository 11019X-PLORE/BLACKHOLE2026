package frc.robot.subsystems.extension;

import edu.wpi.first.math.util.Units;
import frc.robot.Ports;

public class ExtensionConstants {

  public static final int kExtensionId = Ports.kExtension;
  public static final boolean kExtensionInverted = false;

  public static final double kExtensionRadius = 0.0124; // TODO
  public static final double kExtensionGearRatio = 3.0; // TODO

  public static final double kP = 40.0; // 20
  public static final double kI = 0.0;
  public static final double kD = 0.1; // 0.7
  public static final double kG = 0.0; //
  public static final double kS = 0.3; // 0.3
  public static final double kV = 3.0; // 0.5
  public static final double kA = 0.5; // 0.2

  public static final double kVelocityRadPerSec = Units.degreesToRadians(400.0); // 400
  public static final double kAccelerationRadPerSecSq = Units.degreesToRadians(800.0); // 600

  public static final double kExtensionMinAngle = Units.degreesToRadians(-20.0); // TODO:零位位于水平位置0.0
  public static final double kExtensionMaxAngle = Units.degreesToRadians(130.0); // 150.0
  public static final double kExtensionInitialAngle = Units.degreesToRadians(123.2); // 123.0
  public static final double kExtensionDeployAngle = Units.degreesToRadians(0.0); // 展开位置//
  public static final double kExtensionStowerAngle = Units.degreesToRadians(90.0); //
  public static final double kExtensionShakeAngle = Units.degreesToRadians(45.0);

  public static final double kExtensionoffset =
      Units.degreesToRadians(0.0); // TODO: offset for Extension angle
  public static final double kExtensiontoleranceDeg = Units.degreesToRadians(1.0);

  public static final double kFixAngle = Units.degreesToRadians(90.0);

  public static final double kShakeFrequency = 1.0;
}
