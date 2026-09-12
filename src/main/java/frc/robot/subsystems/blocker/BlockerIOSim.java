package frc.robot.subsystems.blocker;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class BlockerIOSim implements BlockerIO {
  private static final DCMotor motorModel = DCMotor.getKrakenX44(1);
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          motorModel,
          BlockerConstants.kBlockerGearRatio,
          0.002,
          0.2,
          BlockerConstants.kMinAngle,
          BlockerConstants.kMaxAngle,
          false,
          BlockerConstants.kInitialAngle);

  private final PIDController controller = new PIDController(5.0, 0, 0, Constants.loopPeriodSecs);

  private double appliedVolts = 0.0;
  private boolean closedLoop = false;

  public BlockerIOSim() {
    sim.setState(BlockerConstants.kInitialAngle, 0.0);
  }

  @Override
  public void updateInputs(BlockerIOInputs inputs) {
    if (closedLoop) {
      appliedVolts = MathUtil.clamp(controller.calculate(sim.getAngleRads()), -12.0, 12.0);
    } else {
      appliedVolts = 0.0;
    }
    sim.setInputVoltage(appliedVolts);
    sim.update(Constants.loopPeriodSecs);

    inputs.connected = true;
    inputs.positionRads = sim.getAngleRads();
    inputs.velocityRadsPerSec = sim.getVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.torqueCurrentAmps = sim.getCurrentDrawAmps();
    inputs.tempCelsius = 0.0;
  }

  @Override
  public void setPID(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
    controller.setPID(kP, kI, kD);
  }

  @Override
  public void applyOutputs(BlockerIOOutputs outputs) {
    switch (outputs.mode) {
      case BRAKE, COAST -> {
        closedLoop = false;
        controller.reset();
      }
      case POSITION -> {
        controller.setSetpoint(outputs.positionRads);
        closedLoop = true;
      }
    }
  }
}
