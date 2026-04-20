package frc.robot.subsystems.triggers;

import static edu.wpi.first.units.Units.Amps;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;

public class TriggersIOReal implements TriggersIO {
  private final TalonFX talon_left;
  private final TalonFX talon_right;

  private final CANrange leftCANrange;
  private final CANrange rightCANrange;

  // 状态信号以便通过 IO 层读取
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> temp;
  private final StatusSignal<Boolean> leftLimitSwitch;
  private final StatusSignal<Boolean> rightLimitSwitch;

  // 控制请求
  private final VelocityTorqueCurrentFOC velocityControl = new VelocityTorqueCurrentFOC(0.0);
  private final VoltageOut voltageControl = new VoltageOut(0);
  private final NeutralOut coastControl = new NeutralOut();

  private final HardwareLimitSwitchConfigs limitLeftConfigs;
  private final HardwareLimitSwitchConfigs limitRightConfigs;

  private boolean feedingLeft = true;
  private double lastChangeTime = 0.0;

  public TriggersIOReal() {
    talon_left = new TalonFX(TriggersConstants.kTestLeftTriggersId);
    talon_right = new TalonFX(TriggersConstants.kTestRightTriggersId);
    final TalonFXConfiguration config_left =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(
                        TriggersConstants.kTestLeftTriggersInverted
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive))
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(TriggersConstants.kTriggersGearRatio))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(80.0))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(40))
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLowerLimit(Amps.of(60)) // 允许短时间更高电流
                    .withSupplyCurrentLowerTime(0.1)
                    .withSupplyCurrentLimitEnable(true));

    tryUntilOk(5, () -> talon_left.getConfigurator().apply(config_left));

    TalonFXConfiguration config_right =
        config_left
            .clone()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(
                        TriggersConstants.kTestRightTriggersInverted
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive));

    tryUntilOk(5, () -> talon_right.getConfigurator().apply(config_right));

    leftCANrange = new CANrange(TriggersConstants.kTestLeftLimitSwitchId);
    rightCANrange = new CANrange(TriggersConstants.kTestRightLimitSwitchId);

    final CANrangeConfiguration canRangeConfig =
        new CANrangeConfiguration()
            .withProximityParams(
                new ProximityParamsConfigs()
                    .withProximityThreshold(0.3)
                    .withProximityHysteresis(0.01)); // TODO
    tryUntilOk(5, () -> leftCANrange.getConfigurator().apply(canRangeConfig));
    tryUntilOk(5, () -> rightCANrange.getConfigurator().apply(canRangeConfig));

    limitLeftConfigs = new HardwareLimitSwitchConfigs();
    limitLeftConfigs.ForwardLimitSource = ForwardLimitSourceValue.RemoteCANcoder;
    limitLeftConfigs.ForwardLimitRemoteSensorID = leftCANrange.getDeviceID();

    limitRightConfigs = new HardwareLimitSwitchConfigs();
    limitRightConfigs.ForwardLimitSource = ForwardLimitSourceValue.RemoteCANcoder;
    limitRightConfigs.ForwardLimitRemoteSensorID = rightCANrange.getDeviceID();

    // 初始化信号
    position = talon_left.getPosition();
    velocity = talon_left.getVelocity();
    appliedVolts = talon_left.getMotorVoltage();
    supplyCurrent = talon_left.getSupplyCurrent();
    torqueCurrent = talon_left.getTorqueCurrent();
    temp = talon_left.getDeviceTemp();

    leftLimitSwitch = leftCANrange.getIsDetected();
    rightLimitSwitch = rightCANrange.getIsDetected();

    // 优化 CAN 总线带宽，将这些信号设为高频同步更新
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        position,
        velocity,
        appliedVolts,
        supplyCurrent,
        torqueCurrent,
        leftLimitSwitch,
        rightLimitSwitch);
  }

  @Override
  public void updateInputs(TriggersIOInputs inputs) {
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
    inputs.feedingLeft = feedingLeft;
    inputs.leftLimitSwitch = leftLimitSwitch.getValue();
    inputs.rightLimitSwitch = rightLimitSwitch.getValue();
    inputs.distance = leftCANrange.getDistance().getValueAsDouble();
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
    tryUntilOk(5, () -> talon_left.getConfigurator().apply(cfg));
    tryUntilOk(5, () -> talon_right.getConfigurator().apply(cfg));
  }

  @Override
  public void applyOutputs(TriggersIOOutputs outputs) {
    BaseStatusSignal.refreshAll(rightLimitSwitch, leftLimitSwitch);

    if (Timer.getFPGATimestamp() - lastChangeTime > TriggersConstants.minSwitchPeriod) {
      if (feedingLeft && (!leftLimitSwitch.getValue())) {
        feedingLeft = false;
      }
      if ((!feedingLeft) && (!rightLimitSwitch.getValue())) {
        feedingLeft = true;
      }
      lastChangeTime = Timer.getFPGATimestamp();
    }

    switch (outputs.mode) {
      case COAST -> {
        talon_left.setControl(coastControl);
        talon_right.setControl(coastControl);
      }
      case VELOCITY -> {
        talon_right.setControl(
            velocityControl.withVelocity(Units.radiansToRotations(outputs.velocityRadsPerSec)));
        talon_left.setControl(
            velocityControl.withVelocity(Units.radiansToRotations(outputs.velocityRadsPerSec)));

        if (feedingLeft) {
          talon_left
              .getConfigurator()
              .apply(
                  new HardwareLimitSwitchConfigs()); // check if this can disable the limit switch
          talon_right.getConfigurator().apply(limitLeftConfigs);
        } else {
          talon_right.getConfigurator().apply(new HardwareLimitSwitchConfigs());
          talon_left.getConfigurator().apply(limitRightConfigs);
        }
      }
      case VOLTAGE -> {
        if (feedingLeft) {
          talon_right.setControl(voltageControl.withOutput(outputs.volts));
          talon_left.setControl(coastControl);
        } else {
          talon_left.setControl(voltageControl.withOutput(outputs.volts));
          talon_right.setControl(coastControl);
        }
      }
    }
  }
}
