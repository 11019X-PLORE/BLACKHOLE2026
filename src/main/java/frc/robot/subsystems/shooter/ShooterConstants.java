package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class ShooterConstants {
  public static Transform3d robotToTurret = new Transform3d(-5.0, 5.0, 0.0, Rotation3d.kZero);
  public static Transform3d turretToCamera =
      new Transform3d(
          -0.1314196, 0.0, 0.2770674, new Rotation3d(0.0, Units.degreesToRadians(-22.5), 0.0));

  private ShooterConstants() {}
}
