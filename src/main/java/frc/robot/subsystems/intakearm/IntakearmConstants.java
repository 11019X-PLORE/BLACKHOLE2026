package frc.robot.subsystems.intakearm;

import edu.wpi.first.math.util.Units;
import frc.robot.Ports;

public class IntakearmConstants {

  public static final int kIntakearmId = Ports.kIntakeArm;
  public static final boolean kIntakearmInverted = true;

  public static final double kIntakearmRadius = 0.241025; // TODO
  public static final double kIntakearmGearRatio = 320.0 / 3.0; // TODO

  public static final double kP = 40.0; // 20
  public static final double kI = 0.0;
  public static final double kD = 0.1; // 0.7
  public static final double kG = 0.0; //
  public static final double kS = 0.3; // 0.3
  public static final double kV = 3.0; // 0.5
  public static final double kA = 0.5; // 0.2

  public static final double kVelocityRadPerSec = Units.degreesToRadians(400.0); // 400
  public static final double kAccelerationRadPerSecSq = Units.degreesToRadians(800.0); // 600

  public static final double kIntakearmMinAngle = Units.degreesToRadians(-20.0); // TODO:零位位于水平位置0.0
  public static final double kIntakearmMaxAngle = Units.degreesToRadians(130.0); // 150.0
  public static final double kIntakearmInitialAngle = Units.degreesToRadians(123.2); // 123.0
  public static final double kIntakearmDeployAngle = Units.degreesToRadians(0.0); // 展开位置//
  public static final double kIntakearmStowerAngle = Units.degreesToRadians(90.0); //
  public static final double kIntakearmShakeAngle = Units.degreesToRadians(45.0);

  public static final double kIntakearmoffset =
      Units.degreesToRadians(0.0); // TODO: offset for Intakearm angle
  public static final double kIntakearmtoleranceDeg = Units.degreesToRadians(1.0);

  public static final double kFixAngle = Units.degreesToRadians(90.0);
  public static final double kHoldVoltage = -1.0;

  public static final double kShakeFrequency = 1.0;
}
