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

public class L3 extends SequentialCommandGroup {
  public L3(RobotContainer c, Supplier<Boolean> isRightSide) {
    Intake intake = c.getIntake();
    Turret turret = c.getTurret();
    Hood hood = c.getHood();
    Extension extension = c.getExtension();
    Drive drive = c.getDrive();

    addCommands(
        Commands.runOnce(() -> drive.setYFlipped(isRightSide)),
        Commands.parallel(turret.zeroCommand(), hood.zeroCommand(), extension.zeroCommand()),
        drive.resetOdomToPath("L4"),
        Commands.deadline(
            drive.generatePath("L4"),
            intake.setGoalCommand(Intake.IntakeGoal.INTAKE),
            extension.setGoalCommand(Extension.ExtensionGoal.DEPLOYED),
            SuperstructureFactory.activeShooting(c),
            SuperstructureFactory.runIndexerIntake(c)),
        Commands.parallel(intake.setGoalCommand(Intake.IntakeGoal.STOP)).withTimeout(0.1),
        Commands.parallel(
            Commands.deadline(
                drive.generatePath("L5"),
                SuperstructureFactory.shoot(c),
                SuperstructureFactory.feeding(c)),
            intake.setGoalCommand(Intake.IntakeGoal.STOW),
            extension.setGoalCommand(Extension.ExtensionGoal.FEEDING)),
        Commands.parallel(SuperstructureFactory.stopFeeding(c)).withTimeout(0.1),
        Commands.deadline(
            drive.generatePath("L6"),
            intake.setGoalCommand(Intake.IntakeGoal.INTAKE),
            extension.setGoalCommand(Extension.ExtensionGoal.DEPLOYED),
            SuperstructureFactory.activeShooting(c),
            SuperstructureFactory.runIndexerIntake(c)),
        intake.setGoalCommand(Intake.IntakeGoal.STOP).withTimeout(0.1),
        Commands.parallel(
            Commands.deadline(
                drive.generatePath("L7"),
                SuperstructureFactory.shoot(c),
                SuperstructureFactory.feeding(c)),
            intake.setGoalCommand(Intake.IntakeGoal.STOW),
            extension.setGoalCommand(Extension.ExtensionGoal.FEEDING)),
        Commands.parallel(
                SuperstructureFactory.activeShooting(c), SuperstructureFactory.stopFeeding(c))
            .withTimeout(0.1));
  }
}
