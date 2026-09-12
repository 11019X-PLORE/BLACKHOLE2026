package frc.robot.subsystems.indexer;

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
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class IndexerIOReal implements IndexerIO {
  private final TalonFX indexerTalon;
  private final TalonFX triggerTalon;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> temp;

  // Voltage velocity control with FOC disabled (FOC requires a Phoenix Pro license).
  private final VelocityVoltage velocityControl = new VelocityVoltage(0.0).withEnableFOC(false);
  private final NeutralOut coastControl = new NeutralOut();

  public IndexerIOReal(
      int indexerId, boolean clockwisePositive, int triggerId, boolean triggerClockwisePositive) {
    indexerTalon = new TalonFX(indexerId);
    triggerTalon = new TalonFX(triggerId);
    final TalonFXConfiguration indexerConfig =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast)
                    .withInverted(
                        clockwisePositive
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive))
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(IndexerConstants.kIndexerGearRatio))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(Amps.of(IndexerConstants.kIndexerSupplyCurrentLimit))
                    .withSupplyCurrentLimitEnable(true));

    final TalonFXConfiguration triggerConfig =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast)
                    .withInverted(
                        triggerClockwisePositive
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive))
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(IndexerConstants.kIndexerGearRatio))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(Amps.of(IndexerConstants.kIndexerSupplyCurrentLimit))
                    .withSupplyCurrentLimitEnable(true));

    tryUntilOk(5, () -> indexerTalon.getConfigurator().apply(indexerConfig));
    tryUntilOk(5, () -> triggerTalon.getConfigurator().apply(triggerConfig));

    position = indexerTalon.getPosition();
    velocity = indexerTalon.getVelocity();
    appliedVolts = indexerTalon.getMotorVoltage();
    supplyCurrent = indexerTalon.getSupplyCurrent();
    torqueCurrent = indexerTalon.getTorqueCurrent();
    temp = indexerTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVolts, supplyCurrent, torqueCurrent);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(
                position, velocity, appliedVolts, supplyCurrent, torqueCurrent, temp)
            .isOK();
    inputs.positionRads = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.velocityRadsPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVoltage = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.tempCelsius = temp.getValueAsDouble();
  }

  @Override
  public void setIndexerPID(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
    Slot0Configs cfg = new Slot0Configs();
    cfg.kP = kP;
    cfg.kI = kI;
    cfg.kD = kD;
    cfg.kS = kS;
    cfg.kV = kV;
    cfg.kA = kA;
    tryUntilOk(5, () -> indexerTalon.getConfigurator().apply(cfg));
  }

  @Override
  public void setTriggerPID(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
    Slot0Configs cfg = new Slot0Configs();
    cfg.kP = kP;
    cfg.kI = kI;
    cfg.kD = kD;
    cfg.kS = kS;
    cfg.kV = kV;
    cfg.kA = kA;
    tryUntilOk(5, () -> triggerTalon.getConfigurator().apply(cfg));
  }

  @Override
  public void applyOutputs(IndexerIOOutputs outputs) {
    switch (outputs.mode) {
      case COAST -> {
        indexerTalon.setControl(coastControl);
        triggerTalon.setControl(coastControl);
      }
      case VELOCITY -> {
        indexerTalon.setControl(
            velocityControl.withVelocity(
                Units.radiansToRotations(outputs.indexerVelocityRadsPerSec)));
        triggerTalon.setControl(
            velocityControl.withVelocity(
                Units.radiansToRotations(outputs.triggerVelocityRadsPerSec)));
      }
    }
  }
}
