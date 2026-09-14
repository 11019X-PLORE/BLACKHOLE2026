package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.BatteryLogger;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTracer;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private RobotContainer robotContainer;

  public static final BatteryLogger batteryLogger = new BatteryLogger();
  private final BatteryIOInputsAutoLogged batteryInputs = new BatteryIOInputsAutoLogged();
  // Thread m_visionThread;

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    CommandScheduler.getInstance()
        .onCommandInitialize(
            command ->
                Logger.recordOutput("CommandEvents/LastEvent", "Started: " + command.getName()));

    CommandScheduler.getInstance()
        .onCommandInterrupt(
            command ->
                Logger.recordOutput(
                    "CommandEvents/LastEvent", "Interrupted: " + command.getName()));

    CommandScheduler.getInstance()
        .onCommandFinish(
            command ->
                Logger.recordOutput("CommandEvents/LastEvent", "Finished: " + command.getName()));

    robotContainer = new RobotContainer();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Update battery inputs
    batteryInputs.batteryVoltage = RobotController.getBatteryVoltage();
    batteryInputs.rioCurrent = RobotController.getInputCurrent();
    batteryInputs.macMiniCurrent = 0.0;
    Logger.processInputs("BatteryLogger", batteryInputs);
    batteryLogger.setBatteryVoltage(batteryInputs.batteryVoltage);
    batteryLogger.setRioCurrent(batteryInputs.rioCurrent);
    batteryLogger.setMacMiniCurrent(batteryInputs.macMiniCurrent);
    LoggedTracer.record("BatteryLogger/Periodic");

    FullSubsystem.runAllUpdateInputsPeriodic();
    CommandScheduler.getInstance().run();
    FullSubsystem.runAllPeriodicAfterScheduler();
    FullSubsystem.runAllExecutePeriodic();
    batteryLogger.periodicAfterScheduler();
    LoggedTracer.record("Robot/AfterScheduler");
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    robotContainer.fuelSim.updateSim();
  }

  @AutoLog
  public static class BatteryIOInputs {
    public double batteryVoltage = 12.0;
    public double rioCurrent = 0.0;
    public double macMiniCurrent = 0.0;
  }
}
