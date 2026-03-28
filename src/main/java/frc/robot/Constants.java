package frc.robot;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;

import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.Autopilot;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final double loopPeriodSecs = 0.02;
  public static final boolean tuningMode = true;
  public static boolean disableHAL = false; //
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public final class AutoPilotConstants {

    private static final APConstraints kConstraints =
        new APConstraints().withAcceleration(12.0).withJerk(4.0);
    public static final Angle kMaxAngularVelocity = Degrees.of(360.0);
    private static final APProfile kProfile =
        new APProfile(kConstraints)
            .withErrorXY(Centimeters.of(3))
            .withErrorTheta(Degrees.of(1))
            .withBeelineRadius(Centimeters.of(1));
    public static final Autopilot kAutopilot = new Autopilot(kProfile);
  }
}
