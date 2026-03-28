package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoUtil {
  public static Command followPath(String name) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(name);
      return AutoBuilder.followPath(path);
    } catch (Exception e) {
      DriverStation.reportError("Load path error: " + name, e.getStackTrace());
      return Commands.none();
    }
  }

  public static Command followChoreoPath(String name, boolean mirror) {
    try {
      PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(name);

      if (mirror) {
        path = path.mirrorPath();
      }

      return AutoBuilder.followPath(path);

    } catch (Exception e) {
      DriverStation.reportError("Load path error: " + name, e.getStackTrace());
      return Commands.none();
    }
  }

  public static Command followChoreoPath(String name) {
    return followChoreoPath(name, false);
  }
}
