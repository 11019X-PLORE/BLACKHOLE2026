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

public class L6 extends SequentialCommandGroup {
  public L6(RobotContainer c, Supplier<Boolean> isRightSide) {
    Intake intake = c.getIntake();
    Turret turret = c.getTurret();
    Hood hood = c.getHood();
    Extension extension = c.getExtension();
    Drive drive = c.getDrive();

    addCommands(
        Commands.runOnce(() -> drive.setYFlipped(isRightSide)),
        Commands.parallel(turret.zeroCommand(), hood.zeroCommand(), extension.zeroCommand()),
        drive.resetOdomToPath("L14"),
        Commands.deadline(
            Commands.sequence(drive.generatePath("L14"), drive.generatePath("L15")),
            intake.setGoalCommand(Intake.IntakeGoal.INTAKE),
            extension.setGoalCommand(Extension.ExtensionGoal.DEPLOYED),
            Commands.sequence(
                Commands.parallel(
                    SuperstructureFactory.activeShooting(c).withTimeout(2),
                    SuperstructureFactory.runIndexerIntake(c)),
                Commands.parallel(
                    SuperstructureFactory.feeding(c), SuperstructureFactory.pass(c)))),
        Commands.deadline(drive.generatePath("L16"), SuperstructureFactory.activeShooting(c)),
        SuperstructureFactory.stopFeeding(c));
  }
}
