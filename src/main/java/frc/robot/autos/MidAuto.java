// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.extension.Extension;
import frc.robot.subsystems.hanger.Hanger;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.robot.util.geometry.AllianceFlipUtil;

public class MidAuto extends SequentialCommandGroup {
  public MidAuto(
      Superstructure superstructure,
      Intake intake,
      Turret turret,
      Hood hood,
      Extension extension,
      Hanger hanger,
      Drive drive) {

    addCommands(
        Commands.parallel(
            turret.zeroCommand(),
            hood.zeroCommand(),
            extension.zeroCommand(),
            hanger.zeroCommand()),
        resetOdomToPath("M1", drive).withTimeout(0.2),
        Commands.parallel(
            generatePath("M1"),
            intake.setGoalCommand(Intake.IntakeGoal.INTAKE),
            extension.setGoalCommand(Extension.ExtensionGoal.DEPLOYED)),
        superstructure.setGoal(SuperstructureState.SHOOTING).withTimeout(2.5),
        superstructure.setGoal(SuperstructureState.ACTIVESHOOTING).withTimeout(0.2),
        Commands.parallel(generatePath("M2"), superstructure.setGoal(SuperstructureState.INTAKE)),
        superstructure.setGoal(SuperstructureState.SHOOTING).withTimeout(2.0),
        Commands.parallel(
                intake.setGoalCommand(Intake.IntakeGoal.STOW),
                extension.setGoalCommand(Extension.ExtensionGoal.SHAKE))
            .withTimeout(0.2),
        superstructure.setGoal(SuperstructureState.SHOOTING).withTimeout(1.0),
        Commands.parallel(
                intake.setGoalCommand(Intake.IntakeGoal.STOP),
                extension.setGoalCommand(Extension.ExtensionGoal.STOWED))
            .withTimeout(0.2),
        generatePath("M3"),
        superstructure.setGoal(SuperstructureState.SHOOTING).withTimeout(3.0));
  }

  public static Command resetOdomToPath(String pathName, Drive drive) {
    return Commands.runOnce(
        () -> {
          try {
            PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(pathName);
            // 获取起点并从 Optional 中取出，然后应用红蓝翻转
            Pose2d startPose = path.getStartingHolonomicPose().orElse(new Pose2d());
            drive.setPose(AllianceFlipUtil.apply(startPose));
          } catch (Exception e) {
            DriverStation.reportError(
                "Failed to reset odom for path: " + pathName, e.getStackTrace());
          }
        },
        drive);
  }

  public static Command generatePath(String pathName) {
    PathPlannerPath path;
    try {
      path = PathPlannerPath.fromChoreoTrajectory(pathName);
    } catch (Exception e) {
      DriverStation.reportError("Failed to load path: " + pathName, e.getStackTrace());
      return Commands.none();
    }

    return AutoBuilder.followPath(path);
  }
}
