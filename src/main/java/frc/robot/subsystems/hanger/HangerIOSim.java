package frc.robot.subsystems.hanger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class HangerIOSim implements HangerIO {
  private static final DCMotor motorModel = DCMotor.getKrakenX44(1);
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          motorModel, 1.0, .004, .33, -1.0, Units.degreesToRadians(60), false, 0);

  private final PIDController controller = new PIDController(0.003, 0, 0, Constants.loopPeriodSecs);

  private double currentOutput = 0.0;
  private double appliedVolts = 0.0;
  private boolean currentControl = false;

  public HangerIOSim() {}

  @Override
  public void updateInputs(HangerIOInputs inputs) {
    if (currentControl) {
      appliedVolts = motorModel.getVoltage(currentOutput, sim.getVelocityRadPerSec());
    } else {
      appliedVolts = 0.0;
    }

    // Update sim state
    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    sim.update(Constants.loopPeriodSecs);

    inputs.motorConnected = true;
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
  public void applyOutputs(HangerIOOutputs outputs) {

    // controller.setP(outputs.kP);
    // controller.setD(outputs.kD);
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
        currentOutput = controller.calculate(sim.getAngleRads(), outputs.positionRad);

        currentControl = true;
      }
      case VOLTAGE -> {
        currentOutput = outputs.appliedVolts;
        sim.setInputVoltage(currentOutput); // 施加一个小的反向电压来检测堵转.
        currentControl = true;
      }
    }
  }
}
