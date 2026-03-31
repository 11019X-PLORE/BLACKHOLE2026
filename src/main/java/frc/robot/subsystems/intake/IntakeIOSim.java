package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class IntakeIOSim implements IntakeIO {
  private static final DCMotor motorModel = DCMotor.getKrakenX60(1);
  private static final DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(motorModel, .025, IntakeConstants.kIntakeGearRatio),
          motorModel);
  private PIDController controller = new PIDController(0.5, 0, 0, Constants.loopPeriodSecs);
  private double currentOutput = 0.0;
  private double currentOutputAsVolt = 0.0;
  private double appliedVolts = 0.0;

  public IntakeIOSim() {}

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    currentOutputAsVolt = motorModel.getVoltage(currentOutput, sim.getAngularVelocityRadPerSec());
    appliedVolts = currentOutputAsVolt;

    // Update sim state
    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    sim.update(Constants.loopPeriodSecs);

    inputs.connected = true;
    inputs.positionRads = sim.getAngularPositionRad();
    inputs.velocityRadsPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVoltage = appliedVolts;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.torqueCurrentAmps = currentOutput;
    inputs.tempCelsius = 0.0;
  }

  @Override
  public void applyOutputs(IntakeIOOutputs outputs) {
    if (outputs.mode == IntakeIOOutputMode.COAST) {
      currentOutput = 0.0;
    } else {
      currentOutput =
          controller.calculate(sim.getAngularVelocityRadPerSec(), outputs.velocityRadsPerSec);
    }
  }
}
