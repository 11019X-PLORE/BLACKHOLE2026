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

    inputs.indexRightconnected = true;
    inputs.indexRightvelocityRadsPerSec = indexerSim.getAngularVelocityRadPerSec();
    // Triggers sim update
    double triggersOutputAsVolt =
        triggersMotorModel.getVoltage(
            triggersCurrentOutput, triggersSim.getAngularVelocityRadPerSec());
    triggersAppliedVolts = triggersOutputAsVolt;
    triggersSim.setInputVoltage(MathUtil.clamp(triggersAppliedVolts, -12.0, 12.0));
    triggersSim.update(Constants.loopPeriodSecs);

    inputs.triggersConnected = true;
    inputs.triggersVelocityRadsPerSec = triggersSim.getAngularVelocityRadPerSec();
    inputs.triggersRightconnected = true;
    inputs.triggersRightVelocityRadsPerSec = triggersSim.getAngularVelocityRadPerSec();
    inputs.feedingLeft = true;
    inputs.leftLimitSwitchconnected = true;
    inputs.rightLimitSwitchconnected = true;
    inputs.leftLimitSwitch = false;
    inputs.rightLimitSwitch = false;
    inputs.triggerDistance = 0.0;
  }

  // 同步添加缺少的 Indexer PID 设置方法
  @Override
  public void setPID(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
    indexerController.setPID(kP, kI, kD);
  }

  // 同步添加缺少的 Triggers PID 设置方法
  @Override
  public void setTriggersPID(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
    triggersController.setPID(kP, kI, kD);
  }

  @Override
  public void applyOutputs(IndexerIOOutputs outputs) {
    // === Indexer ===
    switch (outputs.mode) {
      case COAST -> {
        indexerCurrentOutput = 0.0;
      }
      case VELOCITY, FEEDING -> {
        indexerCurrentOutput =
            indexerController.calculate(
                indexerSim.getAngularVelocityRadPerSec(), outputs.velocityRadsPerSec);
      }
    }

    // === Triggers ===
    switch (outputs.triggersMode) {
      case COAST -> {
        triggersCurrentOutput = 0.0;
      }
      case VELOCITY, FEEDING -> {
        triggersCurrentOutput =
            triggersController.calculate(
                triggersSim.getAngularVelocityRadPerSec(), outputs.triggersVelocityRadsPerSec);
      }
    }
  }
}
