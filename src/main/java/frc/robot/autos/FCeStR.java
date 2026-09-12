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

public class FCeStR extends SequentialCommandGroup {
  public FCeStR(RobotContainer c, Supplier<Boolean> isRightSide) {
    Arm arm = c.arm;
    Flywheel flywheel = c.flywheel;
    Indexer indexer = c.indexer;
    Drive drive = c.getDrive();
    Blocker blocker = c.blocker;

    addCommands(
        Commands.runOnce(() -> drive.setYFlipped(isRightSide)),
        drive.resetOdomToPath("FCeSt"),
        new WaitCommand(4),
        Commands.deadline(
            drive.generatePath("FCeSt"),
            arm.setGoalCommand(ArmGoal.INTAKE),
            indexer.setGoalCommand(IndexerGoal.INTAKE),
            flywheel.setGoalCommand(FlywheelGoal.INTAKE),
            Commands.sequence(
                blocker.setGoalCommand(BlockerGoal.CLOSE),
                new WaitCommand(2),
                blocker.setGoalCommand(BlockerGoal.HALFOPEN))),
        Commands.deadline(
            new WaitCommand(5),
            CommandFactory.shoot(c, () -> (0), () -> (0)),
            blocker.shootCommand()),
        Commands.sequence(
            blocker.setGoalCommand(BlockerGoal.CLOSE),
            arm.setGoalCommand(ArmGoal.INTAKE),
            indexer.setGoalCommand(IndexerGoal.INTAKE),
            flywheel.setGoalCommand(FlywheelGoal.INTAKE),
            new WaitCommand(0.5),
            drive.generatePath("StR")));
  }
}
