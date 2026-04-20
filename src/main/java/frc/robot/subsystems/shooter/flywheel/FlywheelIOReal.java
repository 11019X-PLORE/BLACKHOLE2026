package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.Amps;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class FlywheelIOReal implements FlywheelIO {
  private final TalonFX talon;
  private final TalonFX secondTalon;
  // 状态信号以便通过 IO 层读取
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> temp;

  private final StatusSignal<Angle> secondposition;
  private final StatusSignal<AngularVelocity> secondvelocity;
  private final StatusSignal<Voltage> secondappliedVolts;
  private final StatusSignal<Current> secondsupplyCurrent;
  private final StatusSignal<Current> secondtorqueCurrent;
  private final StatusSignal<Temperature> secondtemp;

  // 控制请求
  private final VelocityTorqueCurrentFOC velocityControl = new VelocityTorqueCurrentFOC(0.0);
  private final VoltageOut voltageControl = new VoltageOut(0);
  private final NeutralOut coastControl = new NeutralOut();

  public FlywheelIOReal(int id, boolean isclockwice_Positive) {
    talon = new TalonFX(FlywheelConstants.kFlywheelId);
    secondTalon = new TalonFX(FlywheelConstants.kSecondFlywheel);
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
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(FlywheelConstants.kFlywheelGearRatio))
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
    tryUntilOk(5, () -> secondTalon.getConfigurator().apply(config));
    secondTalon.setControl(new Follower(talon.getDeviceID(), MotorAlignmentValue.Opposed));

    // 初始化信号
    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    torqueCurrent = talon.getTorqueCurrent();
    temp = talon.getDeviceTemp();

    secondposition = secondTalon.getPosition();
    secondvelocity = secondTalon.getVelocity();
    secondappliedVolts = secondTalon.getMotorVoltage();
    secondsupplyCurrent = secondTalon.getSupplyCurrent();
    secondtorqueCurrent = secondTalon.getTorqueCurrent();
    secondtemp = secondTalon.getDeviceTemp();

    // 优化 CAN 总线带宽，将这些信号设为高频同步更新
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        position,
        velocity,
        appliedVolts,
        supplyCurrent,
        torqueCurrent,
        secondposition,
        secondvelocity,
        secondappliedVolts,
        secondsupplyCurrent,
        secondtorqueCurrent);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    // 刷新所有信号
    BaseStatusSignal.refreshAll(
        position,
        velocity,
        appliedVolts,
        supplyCurrent,
        torqueCurrent,
        temp,
        secondposition,
        secondvelocity,
        secondappliedVolts,
        secondsupplyCurrent,
        secondtorqueCurrent);

    inputs.connected = true;
    inputs.positionRads = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.velocityRadsPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVoltage = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.tempCelsius = temp.getValueAsDouble();

    inputs.secondconnected = true;
    inputs.secondpositionRads = Units.rotationsToRadians(secondposition.getValueAsDouble());
    inputs.secondvelocityRadsPerSec = Units.rotationsToRadians(secondvelocity.getValueAsDouble());
    inputs.secondappliedVoltage = secondappliedVolts.getValueAsDouble();
    inputs.secondsupplyCurrentAmps = secondsupplyCurrent.getValueAsDouble();
    inputs.secondtorqueCurrentAmps = secondtorqueCurrent.getValueAsDouble();
    inputs.secondtempCelsius = secondtemp.getValueAsDouble();
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
  public void applyOutputs(FlywheelIOOutputs outputs) {
    switch (outputs.mode) {
      case COAST -> talon.setControl(coastControl);
      case VELOCITY -> {
        talon.setControl(
            velocityControl.withVelocity(Units.radiansToRotations(outputs.velocityRadsPerSec)));
      }
      case VOLTAGE -> {
        talon.setControl(voltageControl.withOutput(outputs.volts));
      }
      case VELOCITY_FOC -> {
        talon.setControl(
            velocityControl
                .withAcceleration(Units.radiansToRotations(outputs.accelerationRadPerSec2))
                .withVelocity(Units.radiansToRotations(outputs.velocityRadsPerSec))
                .withFeedForward(outputs.feedforwardAmps));
      }
    }
  }
}
