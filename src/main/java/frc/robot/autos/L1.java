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

public class L1 extends SequentialCommandGroup {
  public L1(RobotContainer c, Supplier<Boolean> isRightSide) {
    Intake intake = c.getIntake();
    Turret turret = c.getTurret();
    Hood hood = c.getHood();
    Extension extension = c.getExtension();
    Drive drive = c.getDrive();

    addCommands(
        Commands.runOnce(() -> drive.setYFlipped(isRightSide)),
        Commands.parallel(turret.zeroCommand(), hood.zeroCommand(), extension.zeroCommand()),
        drive.resetOdomToPath("L1"),
        Commands.parallel(
            drive.generatePath("L1"),
            intake.setGoalCommand(Intake.IntakeGoal.INTAKE),
            extension.setGoalCommand(Extension.ExtensionGoal.DEPLOYED),
            SuperstructureFactory.runIndexerIntake(c)),
        SuperstructureFactory.activeShooting(c).withTimeout(0.1),
        drive.generatePath("L2"),
        intake.setGoalCommand(Intake.IntakeGoal.STOP).withTimeout(0.1),
        Commands.deadline(
            SuperstructureFactory.shoot(c).withTimeout(4.0), SuperstructureFactory.feeding(c)),
        Commands.parallel(
                SuperstructureFactory.activeShooting(c), SuperstructureFactory.stopFeeding(c))
            .withTimeout(0.1),
        Commands.deadline(
            drive.generatePath("L3"),
            intake.setGoalCommand(Intake.IntakeGoal.INTAKE),
            extension.setGoalCommand(Extension.ExtensionGoal.DEPLOYED),
            SuperstructureFactory.runIndexerIntake(c)),
        intake.setGoalCommand(Intake.IntakeGoal.STOP).withTimeout(0.1),
        Commands.parallel(
            Commands.deadline(
                drive.generatePath("L5"),
                SuperstructureFactory.shoot(c),
                SuperstructureFactory.feeding(c)),
            intake.setGoalCommand(Intake.IntakeGoal.STOW),
            extension.setGoalCommand(Extension.ExtensionGoal.SHAKE)),
        Commands.parallel(
                SuperstructureFactory.activeShooting(c), SuperstructureFactory.stopFeeding(c))
            .withTimeout(0.1));
  }
}
