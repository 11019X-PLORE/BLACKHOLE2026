package frc.robot.subsystems.rollers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class RollerSystemIOSim implements RollerSystemIO {
  private final DCMotorSim sim;
  private final DCMotor gearbox;
  private double appliedVoltage = 0.0;

  public RollerSystemIOSim(DCMotor motorModel, double reduction, double moi) {
    gearbox = motorModel;
    sim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, moi, reduction), motorModel);
  }

  @Override
  public void updateInputs(RollerSystemIOInputs inputs) {
    sim.update(0.02);

    inputs.connected = true;
    inputs.positionRads = sim.getAngularPositionRad();
    inputs.velocityRadsPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.torqueCurrentAmps =
        gearbox.getCurrent(sim.getAngularVelocityRadPerSec(), appliedVoltage);
    inputs.tempCelsius = 0.0;
  }

  @Override
  public void applyOutputs(RollerSystemIOOutputs outputs) {

    if (DriverStation.isDisabled()) {
      appliedVoltage = 0.0;
    } else {
      appliedVoltage = MathUtil.clamp(outputs.appliedVoltage, -12.0, 12.0);
    }
    sim.setInputVoltage(appliedVoltage);
  }
}
