package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.CommandFactories.CommandFactory;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmGoal;
import frc.robot.subsystems.blocker.Blocker;
import frc.robot.subsystems.blocker.Blocker.BlockerGoal;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.Flywheel.FlywheelGoal;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerGoal;
import java.util.function.Supplier;

public class MSmR extends SequentialCommandGroup {
  public MSmR(RobotContainer c, Supplier<Boolean> isRightSide) {
    Arm arm = c.arm;
    Flywheel flywheel = c.flywheel;
    Indexer indexer = c.indexer;
    Drive drive = c.getDrive();
    Blocker blocker = c.blocker;

    addCommands(
        Commands.runOnce(() -> drive.setYFlipped(isRightSide)),
        drive.resetOdomToPath("MSm"),
        Commands.deadline(drive.generatePath("MSm")),
        Commands.deadline(
            new WaitCommand(6),
            CommandFactory.shoot(c, () -> (0), () -> (0)),
            blocker.shootCommand()),
        blocker.setGoalCommand(BlockerGoal.CLOSE),
        arm.setGoalCommand(ArmGoal.ZEROING),
        indexer.setGoalCommand(IndexerGoal.STOP),
        flywheel.setGoalCommand(FlywheelGoal.IDLE),
        new WaitCommand(7.7),
        Commands.deadline(drive.generatePath("SmR")));
  }
}
