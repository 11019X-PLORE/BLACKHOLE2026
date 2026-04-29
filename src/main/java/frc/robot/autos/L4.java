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

public class L4 extends SequentialCommandGroup {
  public L4(RobotContainer c, Supplier<Boolean> isRightSide) {
    Intake intake = c.getIntake();
    Turret turret = c.getTurret();
    Hood hood = c.getHood();
    Extension extension = c.getExtension();
    Drive drive = c.getDrive();

    addCommands(
        Commands.runOnce(() -> drive.setYFlipped(isRightSide)),
        Commands.parallel(turret.zeroCommand(), hood.zeroCommand(), extension.zeroCommand()),
        drive.resetOdomToPath("L8"),
        Commands.deadline(
            drive.generatePath("L8"),
            intake.setGoalCommand(Intake.IntakeGoal.INTAKE),
            extension.setGoalCommand(Extension.ExtensionGoal.DEPLOYED),
            SuperstructureFactory.activeShooting(c),
            SuperstructureFactory.runIndexerIntake(c)),
        Commands.deadline(
            drive.generatePath("L9"),
            SuperstructureFactory.pass(c),
            SuperstructureFactory.feeding(c)),
        Commands.deadline(
            drive.generatePath("L10"),
            intake.setGoalCommand(Intake.IntakeGoal.INTAKE),
            extension.setGoalCommand(Extension.ExtensionGoal.DEPLOYED),
            SuperstructureFactory.activeShooting(c),
            SuperstructureFactory.runIndexerIntake(c)),
        Commands.parallel(
            Commands.deadline(
                drive.generatePath("L11"),
                SuperstructureFactory.shoot(c),
                SuperstructureFactory.feeding(c)),
            intake.setGoalCommand(Intake.IntakeGoal.STOW),
            extension.setGoalCommand(Extension.ExtensionGoal.FEEDING)),
        Commands.parallel(
                SuperstructureFactory.activeShooting(c), SuperstructureFactory.stopFeeding(c))
            .withTimeout(0.1));
  }
}
