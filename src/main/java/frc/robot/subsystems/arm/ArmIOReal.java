package frc.robot.subsystems.arm;

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
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.NeutralOut;
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
import org.littletonrobotics.junction.AutoLogOutput;

public class ArmIOReal implements ArmIO {
  private final TalonFX talon;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> temp;

  // Voltage position control with FOC disabled (FOC requires a Phoenix Pro license).
  private final PositionVoltage positionControl = new PositionVoltage(0.0).withEnableFOC(false);
  private final NeutralOut coastControl = new NeutralOut();
  private final StaticBrake brakeControl = new StaticBrake();

  @AutoLogOutput public static double angleOffset = 0.0; // radians

  public ArmIOReal(int id, boolean clockwisePositive) {
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
                new FeedbackConfigs().withSensorToMechanismRatio(ArmConstants.kArmGearRatio))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(Amps.of(40))
                    .withSupplyCurrentLimitEnable(true))
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(false)
                    .withForwardSoftLimitThreshold(Units.radiansToRotations(ArmConstants.kMaxAngle))
                    .withReverseSoftLimitEnable(false)
                    .withReverseSoftLimitThreshold(
                        Units.radiansToRotations(ArmConstants.kMinAngle)))
            .withVoltage(
                new VoltageConfigs()
                    .withPeakForwardVoltage(ArmConstants.kMaxVoltage)
                    .withPeakReverseVoltage(-ArmConstants.kMaxVoltage));

    tryUntilOk(5, () -> talon.getConfigurator().apply(config));
    resetAngle(ArmConstants.kInitialAngle);

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
  public void updateInputs(ArmIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(
                position, velocity, appliedVolts, supplyCurrent, torqueCurrent, temp)
            .isOK();
    inputs.positionRads = Units.rotationsToRadians(position.getValueAsDouble()) - angleOffset;
    inputs.velocityRadsPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.tempCelsius = temp.getValueAsDouble();
  }

  @Override
  public void applyOutputs(ArmIOOutputs outputs) {
    switch (outputs.mode) {
      case BRAKE -> talon.setControl(brakeControl);
      case COAST -> talon.setControl(coastControl);
      case POSITION -> talon.setControl(
          positionControl
              .withPosition(Units.radiansToRotations(outputs.positionRads + angleOffset))
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
