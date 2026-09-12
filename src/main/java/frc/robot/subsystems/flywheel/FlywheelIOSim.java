package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class FlywheelIOSim implements FlywheelIO {
  private static final DCMotor motorModel = DCMotor.getKrakenX60(1);
  private final DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              motorModel, 0.025, FlywheelConstants.kFlywheelGearRatio),
          motorModel);

  private final PIDController controller = new PIDController(0.5, 0, 0, Constants.loopPeriodSecs);
  private double appliedVolts = 0.0;
  private boolean closedLoop = false;

  public FlywheelIOSim() {}

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    if (closedLoop) {
      appliedVolts =
          MathUtil.clamp(controller.calculate(sim.getAngularVelocityRadPerSec()), -12.0, 12.0);
    } else {
      appliedVolts = 0.0;
    }
    sim.setInputVoltage(appliedVolts);
    sim.update(Constants.loopPeriodSecs);

    inputs.connected = true;
    inputs.positionRads = sim.getAngularPositionRad();
    inputs.velocityRadsPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVoltage = appliedVolts;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.torqueCurrentAmps = sim.getCurrentDrawAmps();
    inputs.tempCelsius = 0.0;
  }

  @Override
  public void setPID(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
    controller.setPID(kP, kI, kD);
  }

  @Override
  public void applyOutputs(FlywheelIOOutputs outputs) {
    switch (outputs.mode) {
      case COAST -> closedLoop = false;
      case VELOCITY -> {
        controller.setSetpoint(outputs.velocityRadsPerSec);
        closedLoop = true;
      }
      case VOLTAGE -> closedLoop = false;
    }
  }
}
