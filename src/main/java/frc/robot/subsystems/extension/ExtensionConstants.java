package frc.robot.subsystems.extension;

import edu.wpi.first.math.util.Units;
import frc.robot.Ports;

public class ExtensionConstants {

  public static final int kExtensionId = Ports.kExtension;
  public static final boolean kExtensionInverted = false;

  public static final double kExtensionRadius = 0.0127; // TODO
  public static final double kExtensionGearRatio = 3.0 * 42.0 / 18.0; // TODO

  public static final double kP = 1000.0; // 20
  public static final double kI = 0.0;
  public static final double kD = 0.1; // 0.7
  public static final double kG = 0.0; //
  public static final double kS = 0.3; // 0.3
  public static final double kV = 3.0; // 0.5
  public static final double kA = 0.5; // 0.2

  public static final double kVelocityRadPerSec = 10 / kExtensionRadius; // 400
  public static final double kAccelerationRadPerSecSq = 20 / kExtensionRadius; // 600

  public static final double kExtensionMinPosition = 0; // TODO:零位位于水平位置0.0
  public static final double kExtensionMaxPosition = 0.270; // 150.0
  public static final double kExtensionInitialPosition = 0; // 123.0
  public static final double kExtensionDeployPosition = 0.275; // 展开位置//
  public static final double kExtensionStowerPosition = 0; //
  public static final double kExtensionShakePosition = 0.1;

  public static final double kExtensionFeedingPosition = 0.14; // 不卡球位置

  public static final double kExtensionoffset =
      Units.degreesToRadians(0.0); // TODO: offset for Extension Position
  public static final double kExtensiontoleranceDeg = 0.02;

  public static final double kFixPosition = 0.1;

  public static final double kShakeFrequency = 1.0;

  public static final double kMaxCapcity = 45; // TODO

  public static final double kStartPushingCapcity = 30; // TODO
  public static final double kStopPushingCapcity = 16; // TODO
}
