package frc.robot.subsystems.rotator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.rollers.RollerSystemIO;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

public class Rotator extends SubsystemBase {
  private static final LoggedTunableNumber rollerIntakeVolts =
      new LoggedTunableNumber("Rotator/Roller/IntakeVolts", 10.0);
  private static final LoggedTunableNumber rollerShootVolts =
      new LoggedTunableNumber("Rotator/Roller/ShootVolts", 9.0);
  private static final LoggedTunableNumber rollerOuttakeVolts =
      new LoggedTunableNumber("Rotator/Roller/OuttakeVolts", -5.0);

  private final RollerSystem roller;

  public enum RotatorGoal {
    INTAKE,
    OUTTAKE,
    SHOOT,
    STOP
  }

  @Getter @Setter @AutoLogOutput private RotatorGoal goal = RotatorGoal.STOP;
  @AutoLogOutput private double rollerVolts = 0.0;

  public Rotator(RollerSystemIO rollerIO) {
    this.roller = new RollerSystem("Rotator roller front", "Rotator/Roller", rollerIO);
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
      case STOP -> {
        rollerVolts = 0.0;
      }
    }
    roller.setVolts(rollerVolts);
  }

  public Command setGoalCommand(RotatorGoal newGoal) {
    return Commands.runOnce(() -> this.goal = newGoal, this);
  }
}
