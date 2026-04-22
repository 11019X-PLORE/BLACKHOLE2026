package frc.robot.subsystems.indexer;

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

public class IndexerIOReal implements IndexerIO {
  // === Indexer Motor ===
  private final TalonFX indexerLeftTalon;
  private final TalonFX indexerRightTalon;

  final TalonFXConfiguration indexerLeftConfig;
  final TalonFXConfiguration indexerRightConfig;

  private final StatusSignal<Angle> indexerPosition;
  private final StatusSignal<AngularVelocity> indexerVelocity;
  private final StatusSignal<Voltage> indexerAppliedVolts;
  private final StatusSignal<Current> indexerSupplyCurrent;
  private final StatusSignal<Current> indexerTorqueCurrent;
  private final StatusSignal<Temperature> indexerTemp;

  private final StatusSignal<AngularVelocity> indexerRightvelocity;

  private final VelocityTorqueCurrentFOC indexerVelocityControl = new VelocityTorqueCurrentFOC(0.0);
  private final NeutralOut indexerCoastControl = new NeutralOut();

  // === Triggers Motors (merged from TriggersIOReal) ===
  private final TalonFX triggersLeft;
  private final TalonFX triggersRight;

  private final CANrange leftCANrange;
  private final CANrange rightCANrange;

  private final StatusSignal<AngularVelocity> triggersVelocity;
  private final StatusSignal<AngularVelocity> triggersRightVelocity;
  private final StatusSignal<Boolean> leftLimitSwitch;
  private final StatusSignal<Boolean> rightLimitSwitch;

  private final VelocityTorqueCurrentFOC triggersVelocityControl =
      new VelocityTorqueCurrentFOC(0.0);
  private final NeutralOut triggersCoastControl = new NeutralOut();

  private final HardwareLimitSwitchConfigs limitLeftConfigs;
  private final HardwareLimitSwitchConfigs limitRightConfigs;

  private boolean feedingLeft = true;
  private double lastChangeTime = 0.0;

  private double lastIndexerLeftStatorCurrentLimit;
  private double lastIndexerRightStatorCurrentLimit;

  public IndexerIOReal() {
    // === Configure Indexer Motor ===
    indexerLeftTalon = new TalonFX(IndexerConstants.kIndexerLeftId);
    indexerRightTalon = new TalonFX(IndexerConstants.kIndexerRightId);
    indexerLeftConfig =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast)
                    .withInverted(
                        IndexerConstants.kIndexerLeftInverted
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive))
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(IndexerConstants.kIndexerGearRatio))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(IndexerConstants.kIndexerStatorCurrentLimit))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(40))
                    .withSupplyCurrentLimitEnable(false)
                    .withSupplyCurrentLowerLimit(Amps.of(60))
                    .withSupplyCurrentLowerTime(0.1)
                    .withSupplyCurrentLimitEnable(true));

    tryUntilOk(5, () -> indexerLeftTalon.getConfigurator().apply(indexerLeftConfig));

    indexerRightConfig =
        indexerLeftConfig
            .clone()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(
                        IndexerConstants.kIndexerRightInverted
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive));
    tryUntilOk(5, () -> indexerRightTalon.getConfigurator().apply(indexerRightConfig));

    lastIndexerLeftStatorCurrentLimit = IndexerConstants.kIndexerStatorCurrentLimit;
    lastIndexerRightStatorCurrentLimit = IndexerConstants.kIndexerStatorCurrentLimit;

    indexerPosition = indexerLeftTalon.getPosition();
    indexerVelocity = indexerLeftTalon.getVelocity();
    indexerAppliedVolts = indexerLeftTalon.getMotorVoltage();
    indexerSupplyCurrent = indexerLeftTalon.getSupplyCurrent();
    indexerTorqueCurrent = indexerLeftTalon.getTorqueCurrent();
    indexerTemp = indexerLeftTalon.getDeviceTemp();

    indexerRightvelocity = indexerRightTalon.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        indexerPosition,
        indexerVelocity,
        indexerAppliedVolts,
        indexerSupplyCurrent,
        indexerTorqueCurrent,
        indexerRightvelocity);

    // === Configure Triggers Motors ===
    triggersLeft = new TalonFX(IndexerConstants.kTriggersLeftId);
    triggersRight = new TalonFX(IndexerConstants.kTriggersRightId);

    final TalonFXConfiguration triggersLeftConfig =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(
                        IndexerConstants.kTriggersLeftInverted
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive))
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(IndexerConstants.kTriggersGearRatio))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120.0))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(40))
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLowerLimit(Amps.of(60))
                    .withSupplyCurrentLowerTime(0.1)
                    .withSupplyCurrentLimitEnable(true));

    tryUntilOk(5, () -> triggersLeft.getConfigurator().apply(triggersLeftConfig));

    TalonFXConfiguration triggersRightConfig =
        triggersLeftConfig
            .clone()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(
                        IndexerConstants.kTriggersRightInverted
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive));

    tryUntilOk(5, () -> triggersRight.getConfigurator().apply(triggersRightConfig));

    // === Configure CANrange sensors ===
    leftCANrange = new CANrange(IndexerConstants.kTriggersLeftLimitSwitchId);
    rightCANrange = new CANrange(IndexerConstants.kTriggersRightLimitSwitchId);

    final CANrangeConfiguration canRangeConfig =
        new CANrangeConfiguration()
            .withProximityParams(
                new ProximityParamsConfigs()
                    .withProximityThreshold(0.3)
                    .withProximityHysteresis(0.01));
    tryUntilOk(5, () -> leftCANrange.getConfigurator().apply(canRangeConfig));
    tryUntilOk(5, () -> rightCANrange.getConfigurator().apply(canRangeConfig));

    limitLeftConfigs = new HardwareLimitSwitchConfigs();
    limitLeftConfigs.ForwardLimitSource = ForwardLimitSourceValue.RemoteCANcoder;
    limitLeftConfigs.ForwardLimitRemoteSensorID = leftCANrange.getDeviceID();

    limitRightConfigs = new HardwareLimitSwitchConfigs();
    limitRightConfigs.ForwardLimitSource = ForwardLimitSourceValue.RemoteCANcoder;
    limitRightConfigs.ForwardLimitRemoteSensorID = rightCANrange.getDeviceID();

    triggersVelocity = triggersLeft.getVelocity();
    triggersRightVelocity = triggersRight.getVelocity();
    leftLimitSwitch = leftCANrange.getIsDetected();
    rightLimitSwitch = rightCANrange.getIsDetected();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, triggersVelocity, triggersRightVelocity, leftLimitSwitch, rightLimitSwitch);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(
                indexerPosition,
                indexerVelocity,
                indexerAppliedVolts,
                indexerSupplyCurrent,
                indexerTorqueCurrent,
                indexerTemp)
            .isOK();

    inputs.positionRads = Units.rotationsToRadians(indexerPosition.getValueAsDouble());
    inputs.velocityRadsPerSec = Units.rotationsToRadians(indexerVelocity.getValueAsDouble());
    inputs.appliedVoltage = indexerAppliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = indexerSupplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = indexerTorqueCurrent.getValueAsDouble();
    inputs.tempCelsius = indexerTemp.getValueAsDouble();

    inputs.indexRightconnected = BaseStatusSignal.refreshAll(indexerRightvelocity).isOK();
    inputs.indexRightvelocityRadsPerSec =
        Units.rotationsToRadians(indexerRightvelocity.getValueAsDouble());

    BaseStatusSignal.refreshAll(triggersVelocity, leftLimitSwitch, rightLimitSwitch);
    inputs.triggersConnected = BaseStatusSignal.refreshAll(triggersVelocity).isOK();
    inputs.triggersVelocityRadsPerSec =
        Units.rotationsToRadians(triggersVelocity.getValueAsDouble());

    inputs.triggersRightconnected = BaseStatusSignal.refreshAll(triggersRightVelocity).isOK();
    inputs.triggersRightVelocityRadsPerSec =
        Units.rotationsToRadians(triggersRightVelocity.getValueAsDouble());

    inputs.feedingLeft = feedingLeft;
    inputs.leftLimitSwitchconnected = BaseStatusSignal.refreshAll(leftLimitSwitch).isOK();
    inputs.rightLimitSwitchconnected = BaseStatusSignal.refreshAll(rightLimitSwitch).isOK();
    inputs.leftLimitSwitch = leftLimitSwitch.getValue();
    inputs.rightLimitSwitch = rightLimitSwitch.getValue();
    inputs.triggerDistance = leftCANrange.getDistance().getValueAsDouble();
  }

  @Override
  public void setPID(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
    Slot0Configs cfg = new Slot0Configs();
    cfg.kP = kP;
    cfg.kI = kI;
    cfg.kD = kD;
    cfg.kS = kS;
    cfg.kV = kV;
    cfg.kA = kA;
    tryUntilOk(5, () -> indexerLeftTalon.getConfigurator().apply(cfg));
    tryUntilOk(5, () -> indexerRightTalon.getConfigurator().apply(cfg));
  }

  @Override
  public void setTriggersPID(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
    Slot0Configs cfg = new Slot0Configs();
    cfg.kP = kP;
    cfg.kI = kI;
    cfg.kD = kD;
    cfg.kS = kS;
    cfg.kV = kV;
    cfg.kA = kA;
    tryUntilOk(5, () -> triggersLeft.getConfigurator().apply(cfg));
    tryUntilOk(5, () -> triggersRight.getConfigurator().apply(cfg));
  }

  private void setIndexerLeftCurrent(double current) {
    if (current == lastIndexerLeftStatorCurrentLimit) {
      return;
    }
    tryUntilOk(
        5,
        () ->
            indexerLeftTalon
                .getConfigurator()
                .apply(indexerLeftConfig.CurrentLimits.withStatorCurrentLimit(current)));
    lastIndexerLeftStatorCurrentLimit = current;
  }

  private void setIndexerRightCurrent(double current) {
    if (current == lastIndexerRightStatorCurrentLimit) {
      return;
    }
    tryUntilOk(
        5,
        () ->
            indexerRightTalon
                .getConfigurator()
                .apply(indexerRightConfig.CurrentLimits.withStatorCurrentLimit(current)));
    lastIndexerRightStatorCurrentLimit = current;
  }

  @Override
  public void applyOutputs(IndexerIOOutputs outputs) {

    // === Apply Triggers Outputs (with feeding direction switching) ===
    BaseStatusSignal.refreshAll(rightLimitSwitch, leftLimitSwitch);

    if (Timer.getFPGATimestamp() - lastChangeTime > IndexerConstants.kTriggersMinSwitchPeriod) {
      if (feedingLeft && (!leftLimitSwitch.getValue())) {
        feedingLeft = false;
      }
      if ((!feedingLeft) && (!rightLimitSwitch.getValue())) {
        feedingLeft = true;
      }
      lastChangeTime = Timer.getFPGATimestamp();
    }

    // === Apply Indexer Outputs ===
    switch (outputs.mode) {
      case COAST -> {
        indexerLeftTalon.setControl(indexerCoastControl);
        indexerRightTalon.setControl(indexerCoastControl);
      }
      case VELOCITY -> {
        setIndexerLeftCurrent(IndexerConstants.kIndexerStatorCurrentLimit);
        setIndexerRightCurrent(IndexerConstants.kIndexerStatorCurrentLimit);

        indexerLeftTalon.setControl(
            indexerVelocityControl.withVelocity(
                Units.radiansToRotations(outputs.velocityRadsPerSec)));
        indexerRightTalon.setControl(
            indexerVelocityControl.withVelocity(
                Units.radiansToRotations(outputs.velocityRadsPerSec)));
      }
      case FEEDING -> {
        if (feedingLeft) {
          setIndexerLeftCurrent(outputs.maxCurrent);
          setIndexerRightCurrent(IndexerConstants.kIndexerStatorCurrentLimit);
          indexerLeftTalon.setControl(
              indexerVelocityControl.withVelocity(
                  Units.radiansToRotations(outputs.maxVelocityRadsPerSec)));
          indexerRightTalon.setControl(
              indexerVelocityControl.withVelocity(
                  Units.radiansToRotations(outputs.velocityRadsPerSec)));
        } else {
          setIndexerLeftCurrent(IndexerConstants.kIndexerStatorCurrentLimit);
          setIndexerRightCurrent(outputs.maxCurrent);
          indexerLeftTalon.setControl(
              indexerVelocityControl.withVelocity(
                  Units.radiansToRotations(outputs.velocityRadsPerSec)));
          indexerRightTalon.setControl(
              indexerVelocityControl.withVelocity(
                  Units.radiansToRotations(outputs.maxVelocityRadsPerSec)));
        }
      }
    }

    switch (outputs.triggersMode) {
      case COAST -> {
        triggersLeft.setControl(triggersCoastControl);
        triggersRight.setControl(triggersCoastControl);
      }
      case VELOCITY -> {
        triggersRight.setControl(
            triggersVelocityControl.withVelocity(
                Units.radiansToRotations(outputs.triggersVelocityRadsPerSec)));
        triggersLeft.setControl(
            triggersVelocityControl.withVelocity(
                Units.radiansToRotations(outputs.triggersVelocityRadsPerSec)));
        triggersLeft.getConfigurator().apply(new HardwareLimitSwitchConfigs());
        triggersRight.getConfigurator().apply(new HardwareLimitSwitchConfigs());
      }
      case FEEDING -> {
        triggersRight.setControl(
            triggersVelocityControl.withVelocity(
                Units.radiansToRotations(outputs.triggersVelocityRadsPerSec)));
        triggersLeft.setControl(
            triggersVelocityControl.withVelocity(
                Units.radiansToRotations(outputs.triggersVelocityRadsPerSec)));

        if (feedingLeft) {
          triggersLeft.getConfigurator().apply(new HardwareLimitSwitchConfigs());
          triggersRight.getConfigurator().apply(limitLeftConfigs);
        } else {
          triggersRight.getConfigurator().apply(new HardwareLimitSwitchConfigs());
          triggersLeft.getConfigurator().apply(limitRightConfigs);
        }
      }
    }
  }
}
