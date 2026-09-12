package frc.robot.subsystems.blocker;

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
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
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

public class BlockerIOReal implements BlockerIO {
  private final TalonFX talon;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> temp;

  // Voltage position control with FOC disabled (FOC requires a Phoenix Pro license).
  private final PositionVoltage positionControl = new PositionVoltage(0.0).withEnableFOC(false);
  private final VoltageOut voltageControl = new VoltageOut(0.0);

  private final NeutralOut coastControl = new NeutralOut();
  private final StaticBrake brakeControl = new StaticBrake();

  public BlockerIOReal(int id, boolean clockwisePositive) {
    talon = new TalonFX(id);

    final TalonFXConfiguration config =
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
                    .withSensorToMechanismRatio(BlockerConstants.kBlockerGearRatio))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(Amps.of(30))
                    .withSupplyCurrentLimitEnable(true))
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(
                        Units.radiansToRotations(BlockerConstants.kMaxAngle))
                    .withReverseSoftLimitEnable(true)
                    .withReverseSoftLimitThreshold(
                        Units.radiansToRotations(BlockerConstants.kMinAngle)));

    tryUntilOk(5, () -> talon.getConfigurator().apply(config));
    resetAngle(BlockerConstants.kInitialAngle);

    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    torqueCurrent = talon.getTorqueCurrent();
    temp = talon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVolts, supplyCurrent, torqueCurrent, temp);
  }

  @Override
  public void updateInputs(BlockerIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(
                position, velocity, appliedVolts, supplyCurrent, torqueCurrent, temp)
            .isOK();
    inputs.positionRads = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.velocityRadsPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.tempCelsius = temp.getValueAsDouble();
  }

  @Override
  public void applyOutputs(BlockerIOOutputs outputs) {
    switch (outputs.mode) {
      case BRAKE -> talon.setControl(brakeControl);
      case COAST -> talon.setControl(coastControl);
      case VOLTAGE -> talon.setControl(voltageControl.withOutput(outputs.voltageOut));

      case POSITION -> talon.setControl(
          positionControl
              .withPosition(Units.radiansToRotations(outputs.positionRads))
              .withVelocity(Units.radiansToRotations(outputs.velocityRadsPerSec)));
    }
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
    cfg.kG = kG;
    tryUntilOk(5, () -> talon.getConfigurator().apply(cfg));
  }

  @Override
  public void resetAngle(double angleRads) {
    talon.getConfigurator().setPosition(Units.radiansToRotations(angleRads));
  }
}
