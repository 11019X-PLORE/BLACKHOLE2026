package frc.robot.subsystems.extension;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class ExtensionIOSim implements ExtensionIO {
  private static final DCMotor motorModel = DCMotor.getKrakenX44(1);
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          motorModel,
          ExtensionConstants.kExtensionGearRatio,
          1.0,
          .33,
          ExtensionConstants.kExtensionMinPosition,
          ExtensionConstants.kExtensionMaxPosition,
          false,
          ExtensionConstants.kExtensionInitialPosition);

  private final PIDController controller =
      new PIDController(10.0, 0, 0.1, Constants.loopPeriodSecs);

  private double currentOutput = 0.0;
  private double appliedVolts = 0.0;
  private boolean currentControl = false;

  public ExtensionIOSim() {}

  @Override
  public void updateInputs(ExtensionIOInputs inputs) {
    if (currentControl) {
      appliedVolts = motorModel.getVoltage(currentOutput, sim.getVelocityRadPerSec());
    } else {
      appliedVolts = 0.0;
    }

    // Update sim state
    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    sim.update(Constants.loopPeriodSecs);

    inputs.connected = true;
    inputs.positionRads = sim.getAngleRads();
    inputs.velocityRadsPerSec = sim.getVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.torqueCurrentAmps = currentOutput;
    inputs.tempCelsius = 0.0;
  }

  @Override
  public void setPID(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
    controller.setP(kP);
    controller.setD(kD);
  }

  @Override
  public void applyOutputs(ExtensionIOOutputs outputs) {

    switch (outputs.mode) {
      case BRAKE -> {
        currentControl = false;
        currentControl = false;
        controller.reset(); // 停止时重置控制器
      }
      case COAST -> {
        currentOutput = 0.0;
        currentControl = true;
      }
      case CLOSED_LOOP -> {
        // 使用 WPILib PIDController 进行计算
        // 它的 calculate 方法内部会自动处理 (Setpoint - Measurement) 的正负号逻辑
        currentOutput = controller.calculate(sim.getAngleRads(), outputs.positionRads);

        currentControl = true;
      }
      case VOLTAGE -> {
        currentOutput = outputs.appliedVolts;
        sim.setInputVoltage(currentOutput);
        currentControl = true;
      }
    }
  }

  @Override
  public void resetPosition(double positionRad) {
    sim.setState(positionRad, 0.0);
  }
}
