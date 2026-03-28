package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.rollers.RollerSystemIO;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

public class Intake extends SubsystemBase {
  private static final LoggedTunableNumber rollerIntakeVolts =
      new LoggedTunableNumber("Intake/Roller/IntakeVolts", 10.0);
  private static final LoggedTunableNumber rollerShootVolts =
      new LoggedTunableNumber("Intake/Roller/ShootVolts", 12.0);
  private static final LoggedTunableNumber rollerOuttakeVolts =
      new LoggedTunableNumber("Intake/Roller/OuttakeVolts", -8.0);
  private static final LoggedTunableNumber rollerStowVolts =
      new LoggedTunableNumber("Intake/Roller/StowVolts", 2.0);

  private final RollerSystem roller;

  public enum IntakeGoal {
    INTAKE,
    SHOOT,
    OUTTAKE,
    STOW,
    STOP
  }

  @Getter @Setter @AutoLogOutput private IntakeGoal goal = IntakeGoal.STOP;
  @AutoLogOutput private double rollerVolts = 0.0;

  public Intake(RollerSystemIO rollerIO) {
    this.roller = new RollerSystem("Intake roller", "Intake/Roller", rollerIO);
  }

  public void periodic() {
    roller.periodic();
    switch (goal) {
      case INTAKE -> {
        rollerVolts = rollerIntakeVolts.get();
      }
      case SHOOT -> {
        rollerVolts = rollerShootVolts.get();
      }
      case OUTTAKE -> {
        rollerVolts = rollerOuttakeVolts.get();
      }
      case STOW -> {
        rollerVolts = rollerStowVolts.get();
      }
      case STOP -> {
        rollerVolts = 0.0;
      }
    }
    roller.setVolts(rollerVolts);
    roller.periodic();
  }

  public Command setGoalCommand(IntakeGoal newGoal) {
    return Commands.runOnce(() -> this.goal = newGoal, this);
  }
}
