package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.rollers.RollerSystemIO;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

public class Indexer extends SubsystemBase {
  private static final LoggedTunableNumber IndexerIntakeVolts =
      new LoggedTunableNumber("Indexer/Roller/IntakeVolts", 8.0);
  private static final LoggedTunableNumber IndexerShootVolts =
      new LoggedTunableNumber("Indexer/Roller/ShootVolts", 9.0);
  private static final LoggedTunableNumber IndexerOuttakeVolts =
      new LoggedTunableNumber("Indexer/Roller/OuttakeVolts", -5.0);

  private final RollerSystem roller;

  public enum IndexerGoal {
    SHOOT,
    INTAKE,
    OUTTAKE,
    STOP
  }

  @Getter @Setter @AutoLogOutput private IndexerGoal goal = IndexerGoal.STOP;
  @AutoLogOutput private double rollerVolts = 0.0;

  public Indexer(RollerSystemIO rollerIO) {
    this.roller = new RollerSystem("Indexer roller", "Indexer/Roller", rollerIO);
  }

  public void periodic() {

    roller.periodic();
    switch (goal) {
      case INTAKE -> {
        rollerVolts = IndexerIntakeVolts.get();
      }
      case SHOOT -> {
        rollerVolts = IndexerShootVolts.get();
      }
      case OUTTAKE -> {
        rollerVolts = IndexerOuttakeVolts.get();
      }
      case STOP -> {
        rollerVolts = 0.0;
      }
    }
    roller.setVolts(rollerVolts);
    roller.periodic();
  }

  public Command setGoalCommand(IndexerGoal newGoal) {
    return Commands.runOnce(() -> this.goal = newGoal, this);
  }
}
