package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Amps;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class TurretIOreal implements TurretIO {
  // hardwares
  private final TalonFX talon;

  // Status Signals
  private final StatusSignal<Angle> turretPosition;
  private final StatusSignal<AngularVelocity> turretVelocity;
  private final StatusSignal<Voltage> turretAppliedVolts;
  private final StatusSignal<Current> turretSupplyCurrent;
  private final StatusSignal<Current> turretTorqueCurrent;
  private final StatusSignal<AngularAcceleration> turretAcceleration;
  private final StatusSignal<Temperature> turretTempCelsius;

  // Control
  private final NeutralOut neutralControl = new NeutralOut();
  private final PositionVoltage positionVoltageControl = new PositionVoltage(0.0);
  private final MotionMagicExpoVoltage mmExpoControl = new MotionMagicExpoVoltage(0.0);
  private double currentKv = 0.0;
  private double currentKs = 0.0;

  public TurretIOreal(int id, boolean isclockwice_Positive) {
    talon = new TalonFX(id);
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
                new FeedbackConfigs().withSensorToMechanismRatio(TurretConstants.kTurretGearRatio))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120.0))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(40)) // 正常最大电流
                    .withSupplyCurrentLowerLimit(Amps.of(60)) // 允许短时间更高电流
                    .withSupplyCurrentLowerTime(0.1)
                    .withSupplyCurrentLimitEnable(true)) // 0.1s 后才真正限流
            .withTorqueCurrent(
                new TorqueCurrentConfigs()
                    .withPeakForwardTorqueCurrent(100.0)
                    .withPeakReverseTorqueCurrent(-100.0))
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(
                        Units.radiansToRotations(TurretConstants.kTurretMaxAngle))
                    .withReverseSoftLimitEnable(true)
                    .withReverseSoftLimitThreshold(
                        Units.radiansToRotations(TurretConstants.kTurretMinAngle)))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(
                        Units.radiansToRotations(TurretConstants.kVelocityRadPerSec))
                    .withMotionMagicAcceleration(
                        Units.radiansToRotations(TurretConstants.kAccelerationRadPerSecSq)));
    ;

    talon.getConfigurator().apply(config);

    resetPosition(TurretConstants.kTurretInitialAngle);
    // Set signals
    turretPosition = talon.getPosition();
    turretVelocity = talon.getVelocity();
    turretAppliedVolts = talon.getMotorVoltage();
    turretSupplyCurrent = talon.getSupplyCurrent();
    turretTorqueCurrent = talon.getTorqueCurrent();
    turretAcceleration = talon.getAcceleration();
    turretTempCelsius = talon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, // TODO: change
        turretPosition,
        turretVelocity,
        turretAppliedVolts,
        turretSupplyCurrent,
        turretTorqueCurrent,
        turretAcceleration,
        turretTempCelsius);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.turretMotorConnected =
        BaseStatusSignal.refreshAll(
                turretPosition,
                turretVelocity,
                turretAppliedVolts,
                turretSupplyCurrent,
                turretTorqueCurrent,
                turretAcceleration,
                turretTempCelsius)
            .isOK();

    inputs.positionRads = Units.rotationsToRadians(turretPosition.getValueAsDouble());
    inputs.velocityRadsPerSec = Units.rotationsToRadians(turretVelocity.getValueAsDouble());
    inputs.turretAppliedVolts = turretAppliedVolts.getValueAsDouble();
    inputs.turretSupplyCurrent = turretSupplyCurrent.getValueAsDouble();
    inputs.turretTorqueCurrent = turretTorqueCurrent.getValueAsDouble();
    inputs.turretTempCelsius = turretTempCelsius.getValueAsDouble();
    inputs.turretAlphaRadsPerSecSquared =
        Units.rotationsToRadians(turretAcceleration.getValueAsDouble());
  }

  @Override
  public void setPID(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
    Slot0Configs config = new Slot0Configs();
    config.kP = kP; // A position error of 0.2 rotations results in 12 V output
    config.kI = kI; // 0
    config.kD = kD; // A velocity error of 1 rps results in 0.5 V output
    config.kS = kS; // static feedforward voltage
    config.kV = kV; // velocity feedforward voltage
    config.kA = kA; // acceleration feedforward voltage
    this.currentKs = kS;
    this.currentKv = kV;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config));
  }

  @Override
  /** 弧度到圈 reset */
  public void resetPosition(double angleRads) {
    angleRads = angleRads / (2 * Math.PI);
    talon.getConfigurator().setPosition(angleRads);
  }

  @Override
  public void applyOutputs(TurretIOOutputs outputs) {
    switch (outputs.mode) {
      case BRAKE -> {
        talon.setControl(neutralControl);
      }
      case COAST -> {
        talon.setControl(neutralControl);
      }
      case CLOSED_LOOP -> {
        double feedForwardAmps =
            (Math.signum(outputs.velocity / (2 * Math.PI)) * currentKs)
                + (outputs.velocity / (2 * Math.PI) * currentKv);
        talon.setControl(
            // positionVoltageControl
            //     .withEnableFOC(true)
            //     .withPosition(outputs.position / (2 * Math.PI))
            //     .withVelocity(outputs.velocity / (2 * Math.PI))
            mmExpoControl
                .withPosition(outputs.position / (2 * Math.PI)) // 目标位置 (Rotations)
                .withFeedForward(feedForwardAmps)
                .withEnableFOC(true) // 注入前馈电流 (Amps)
            );
      }
    }
  }
}
