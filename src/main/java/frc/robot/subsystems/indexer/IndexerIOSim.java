package frc.robot.subsystems.indexer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class IndexerIOSim implements IndexerIO {
  // === Indexer sim ===
  private static final DCMotor indexerMotorModel = DCMotor.getKrakenX60(1);
  private static final DCMotorSim indexerSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              indexerMotorModel, .025, IndexerConstants.kIndexerGearRatio),
          indexerMotorModel);

  private PIDController indexerController = new PIDController(0.5, 0, 0, Constants.loopPeriodSecs);
  private double indexerCurrentOutput = 0.0;
  private double indexerAppliedVolts = 0.0;

  // === Triggers sim ===
  private static final DCMotor triggersMotorModel = DCMotor.getKrakenX60(1);
  private static final DCMotorSim triggersSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              triggersMotorModel, .025, IndexerConstants.kTriggersGearRatio),
          triggersMotorModel);

  private PIDController triggersController = new PIDController(0.5, 0, 0, Constants.loopPeriodSecs);
  private double triggersCurrentOutput = 0.0;
  private double triggersAppliedVolts = 0.0;

  public IndexerIOSim() {}

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    // Indexer sim update
    double indexerOutputAsVolt =
        indexerMotorModel.getVoltage(
            indexerCurrentOutput, indexerSim.getAngularVelocityRadPerSec());
    indexerAppliedVolts = indexerOutputAsVolt;
    indexerSim.setInputVoltage(MathUtil.clamp(indexerAppliedVolts, -12.0, 12.0));
    indexerSim.update(Constants.loopPeriodSecs);

    inputs.connected = true;
    inputs.positionRads = indexerSim.getAngularPositionRad();
    inputs.velocityRadsPerSec = indexerSim.getAngularVelocityRadPerSec();
    inputs.appliedVoltage = indexerAppliedVolts;
    inputs.supplyCurrentAmps = indexerSim.getCurrentDrawAmps();
    inputs.torqueCurrentAmps = indexerCurrentOutput;
    inputs.tempCelsius = 0.0;

    // Triggers sim update
    double triggersOutputAsVolt =
        triggersMotorModel.getVoltage(
            triggersCurrentOutput, triggersSim.getAngularVelocityRadPerSec());
    triggersAppliedVolts = triggersOutputAsVolt;
    triggersSim.setInputVoltage(MathUtil.clamp(triggersAppliedVolts, -12.0, 12.0));
    triggersSim.update(Constants.loopPeriodSecs);

    inputs.triggersConnected = true;
    inputs.triggersVelocityRadsPerSec = triggersSim.getAngularVelocityRadPerSec();
    inputs.feedingLeft = true;
    inputs.leftLimitSwitch = false;
    inputs.rightLimitSwitch = false;
    inputs.triggerDistance = 0.0;
  }

  @Override
  public void applyOutputs(IndexerIOOutputs outputs) {
    // Indexer
    if (outputs.mode == IndexerIOOutputMode.COAST) {
      indexerCurrentOutput = 0.0;
    } else {
      indexerCurrentOutput =
          indexerController.calculate(
              indexerSim.getAngularVelocityRadPerSec(), outputs.velocityRadsPerSec);
    }

    // Triggers
    if (outputs.triggersMode == IndexerIOOutputMode.COAST) {
      triggersCurrentOutput = 0.0;
    } else {
      triggersCurrentOutput =
          triggersController.calculate(
              triggersSim.getAngularVelocityRadPerSec(), outputs.triggersVelocityRadsPerSec);
    }
  }
}
