// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.extension.Extension;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.superstructure.SuperstructureFactory;
import java.util.function.Supplier;

public class M4 extends SequentialCommandGroup {
  public M4(RobotContainer c, Supplier<Boolean> isRightSide) {
    Intake intake = c.getIntake();
    Turret turret = c.getTurret();
    Hood hood = c.getHood();
    Extension extension = c.getExtension();
    Drive drive = c.getDrive();

    addCommands(
        Commands.runOnce(() -> drive.setYFlipped(isRightSide)),
        Commands.parallel(turret.zeroCommand(), hood.zeroCommand(), extension.zeroCommand()),
        drive.resetOdomToPath("M3"),
        Commands.deadline(
            drive.generatePath("M3"),
            intake.setGoalCommand(Intake.IntakeGoal.INTAKE),
            extension.setGoalCommand(Extension.ExtensionGoal.DEPLOYED)),
        Commands.deadline(
            SuperstructureFactory.shoot(c).withTimeout(3.0), SuperstructureFactory.feeding(c)),
        Commands.parallel(
                SuperstructureFactory.activeShooting(c), SuperstructureFactory.stopFeeding(c))
            .withTimeout(0.1),
        Commands.deadline(
            drive.generatePath("M5"),
            intake.setGoalCommand(Intake.IntakeGoal.INTAKE),
            extension.setGoalCommand(Extension.ExtensionGoal.DEPLOYED),
            SuperstructureFactory.runIndexerIntake(c)),
        Commands.parallel(
            Commands.deadline(
                SuperstructureFactory.pass(c).withTimeout(4.0), SuperstructureFactory.feeding(c)),
            intake.setGoalCommand(Intake.IntakeGoal.STOW),
            extension.setGoalCommand(Extension.ExtensionGoal.SHAKE)),
        Commands.parallel(
                SuperstructureFactory.activeShooting(c), SuperstructureFactory.stopFeeding(c))
            .withTimeout(0.1));
  }
}
