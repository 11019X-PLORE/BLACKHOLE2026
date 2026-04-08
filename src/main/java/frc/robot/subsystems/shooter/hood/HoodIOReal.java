package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Amps;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class HoodIOReal implements HoodIO {
  private final TalonFX talon;

  // 状态信号（用于高效读取）
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> temp;

  // 控制请求
  private final PositionVoltage positionControl = new PositionVoltage(0);
  private final PositionTorqueCurrentFOC torqueControl = new PositionTorqueCurrentFOC(0.0);
  private final NeutralOut coastControl = new NeutralOut();
  private final StaticBrake brakeControl = new StaticBrake();

  private double currentKA = 0.0;

  public HoodIOReal(int id, boolean isclockwice_Positive) {
    talon = new TalonFX(HoodConstants.kHoodId);

    final TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(
                        isclockwice_Positive
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive))
            .withFeedback(
                new FeedbackConfigs().withSensorToMechanismRatio(HoodConstants.hoodGearRatio))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(40))
                    .withSupplyCurrentLimitEnable(true))
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(
                        Units.radiansToRotations(HoodConstants.kHoodMaxAngle))
                    .withReverseSoftLimitEnable(true)
                    .withReverseSoftLimitThreshold(
                        Units.radiansToRotations(HoodConstants.kHoodMinAngle)));

    talon.getConfigurator().apply(config);

    resetAngle(HoodConstants.kHoodInitialAngle);

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
  public void updateInputs(HoodIOInputs inputs) {
    // 刷新所有信号
    inputs.motorConnected =
        BaseStatusSignal.refreshAll(
                position, velocity, appliedVolts, supplyCurrent, torqueCurrent, temp)
            .isOK();

    // Phoenix 6 默认单位是 Rotations，需要转为 Rads
    inputs.positionRads = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.velocityRadsPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.tempCelsius = temp.getValueAsDouble();
  }

  @Override
  public void applyOutputs(HoodIOOutputs outputs) {
    switch (outputs.mode) {
      case BRAKE -> talon.setControl(brakeControl);
      case COAST -> talon.setControl(coastControl);
      case CLOSED_LOOP -> {
        talon.setControl(
            positionControl
                .withEnableFOC(true)
                .withPosition(Units.radiansToRotations(outputs.positionRads))
                .withVelocity(Units.radiansToRotations(outputs.velocityRadsPerSec)));
      }
      case POSITION_FOC -> {
        double ffAmps =
            (outputs.accelerationRadPerSec2
                    * HoodConstants.kInertiaHood
                    / HoodConstants.hoodGearRatio)
                / HoodConstants.kT;
        torqueControl
            .withPosition(Units.radiansToRotations(outputs.positionRads))
            .withVelocity(Units.radiansToRotations(outputs.velocityRadsPerSec))
            .withFeedForward(ffAmps);
      }
    }
  }

  @Override
  public void setPID(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
    Slot0Configs cfg = new Slot0Configs();
    cfg.kP = kP; // A position error of 0.2 rotations results in 12 V output
    cfg.kI = kI; // 0
    cfg.kD = kD; // A velocity error of 1 rps results in 0.5 V output
    cfg.kS = kS; // static feedforward voltage
    cfg.kV = kV; // velocity feedforward voltage
    cfg.kG = kG; // gravity feedforward voltage

    this.currentKA = kA; // acceleration feedforward voltage
    tryUntilOk(5, () -> talon.getConfigurator().apply(cfg));
  }

  // @Override
  public void resetAngle(double Radius) {
    talon.getConfigurator().setPosition(Units.radiansToRotations(Radius));
  }
}
