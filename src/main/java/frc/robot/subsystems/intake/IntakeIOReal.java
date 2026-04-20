package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOReal implements IntakeIO {
  private final TalonFX talon;
  // 状态信号以便通过 IO 层读取
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> temp;

  // 控制请求
  private final VelocityTorqueCurrentFOC velocityControl = new VelocityTorqueCurrentFOC(0.0);
  private final VoltageOut voltageControl = new VoltageOut(0);
  private final NeutralOut coastControl = new NeutralOut();

  public IntakeIOReal(int id, boolean isclockwice_Positive) {
    talon = new TalonFX(id);
    final TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast)
                    .withInverted(
                        isclockwice_Positive
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive))
            .withFeedback(
                new FeedbackConfigs().withSensorToMechanismRatio(IntakeConstants.kIntakeGearRatio))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(60.0))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(40))
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLowerLimit(Amps.of(60)) // 允许短时间更高电流
                    .withSupplyCurrentLowerTime(0.1)
                    .withSupplyCurrentLimitEnable(true));

    tryUntilOk(5, () -> talon.getConfigurator().apply(config));

    // 初始化信号
    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    torqueCurrent = talon.getTorqueCurrent();
    temp = talon.getDeviceTemp();

    // 优化 CAN 总线带宽，将这些信号设为高频同步更新
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVolts, supplyCurrent, torqueCurrent);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // 刷新所有信号
    BaseStatusSignal.refreshAll(
        position, velocity, appliedVolts, supplyCurrent, torqueCurrent, temp);

    inputs.connected = true;
    inputs.positionRads = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.velocityRadsPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVoltage = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.tempCelsius = temp.getValueAsDouble();
  }

  @Override
  public void setPID(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
    Slot0Configs cfg = new Slot0Configs();
    cfg.kP = kP; // A position error of 0.2 rotations results in 12 V output
    cfg.kI = kI; // 0
    cfg.kD = kD; // A velocity error of 1 rps results in 0.5 V output
    cfg.kS = kS; // static feedforward voltage
    cfg.kV = kV; // velocity feedforward voltage
    cfg.kA = kA; // acceleration feedforward voltage
    tryUntilOk(5, () -> talon.getConfigurator().apply(cfg));
  }

  @Override
  public void applyOutputs(IntakeIOOutputs outputs) {
    switch (outputs.mode) {
      case COAST -> talon.setControl(coastControl);
      case VELOCITY -> {
        talon.setControl(
            velocityControl.withVelocity(Units.radiansToRotations(outputs.velocityRadsPerSec)));
      }
      case VOLTAGE -> {
        talon.setControl(voltageControl.withOutput(outputs.volts));
      }
    }
  }
}
