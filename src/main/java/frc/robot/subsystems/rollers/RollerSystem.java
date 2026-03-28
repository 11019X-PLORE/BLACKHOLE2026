package frc.robot.subsystems.rollers;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.rollers.RollerSystemIO.RollerSystemIOOutputs;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class RollerSystem extends SubsystemBase {
  private final String name;
  private final String inputsName;
  private final RollerSystemIO io;
  protected final RollerSystemIOInputsAutoLogged inputs = new RollerSystemIOInputsAutoLogged();
  private final RollerSystemIOOutputs outputs = new RollerSystemIOOutputs();

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert disconnected;

  @Setter private double volts = 0.0;
  @Setter private boolean brakeModeEnabled = true;

  public RollerSystem(String name, String inputsName, RollerSystemIO io) {
    this.name = name;
    this.inputsName = inputsName;
    this.io = io;

    disconnected = new Alert(name + " motor disconnected!", Alert.AlertType.kWarning);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(inputsName, inputs);
    disconnected.set(
        // Robot.showHardwareAlerts() &&
        !motorConnectedDebouncer.calculate(inputs.connected)); // TODO: alerts

    // Run roller
    outputs.appliedVoltage = volts;

    // Update brake mode
    outputs.brakeModeEnabled = brakeModeEnabled;
    io.applyOutputs(outputs);
    Logger.recordOutput(inputsName + "/BrakeModeEnabled", brakeModeEnabled);
  }

  // @Override
  // public void periodicAfterScheduler() {
  //   io.applyOutputs(outputs);
  // }

  public double getTorqueCurrent() {
    return inputs.torqueCurrentAmps;
  }

  public double getVelocity() {
    return inputs.velocityRadsPerSec;
  }

  public double getVoltage() {
    return inputs.appliedVoltage;
  }

  public void stop() {
    volts = 0.0;
  }
}
