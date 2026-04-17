package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.extension.Extension;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import java.util.function.Supplier;

public class Test extends SequentialCommandGroup {
  public Test(RobotContainer c, Supplier<Boolean> isRightSide) {
    Intake intake = c.getIntake();
    Turret turret = c.getTurret();
    Hood hood = c.getHood();
    Extension extension = c.getExtension();
    Drive drive = c.getDrive();

    addCommands(
        Commands.runOnce(() -> drive.setYFlipped(isRightSide)),
        Commands.parallel(turret.zeroCommand(), hood.zeroCommand(), extension.zeroCommand()),
        drive.resetOdomToPath("Test1"),
        drive.generatePath("Test1"));
  }
}
