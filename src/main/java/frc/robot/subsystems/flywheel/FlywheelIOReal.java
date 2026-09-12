package frc.robot.subsystems.flywheel;

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
import com.ctre.phoenix6.controls.VelocityVoltage;
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
  private final TalonFX[] talons;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> temp;

  // Voltage control with FOC disabled (FOC requires a Phoenix Pro license).
  private final VelocityVoltage velocityControl = new VelocityVoltage(0.0).withEnableFOC(false);
  private final VoltageOut voltageControl = new VoltageOut(0).withEnableFOC(false);
  private final NeutralOut coastControl = new NeutralOut();

  public FlywheelIOReal(int[] ids, boolean clockwisePositive) {
    talons = new TalonFX[ids.length];
    for (int i = 0; i < ids.length; i++) {
      talons[i] = new TalonFX(ids[i]);
    }

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
                    .withSensorToMechanismRatio(FlywheelConstants.kFlywheelGearRatio))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(Amps.of(40))
                    .withSupplyCurrentLimitEnable(true));

    tryUntilOk(5, () -> talons[0].getConfigurator().apply(config));

    for (int i = 1; i < talons.length; i++) {
      int index = i;
      tryUntilOk(5, () -> talons[index].getConfigurator().apply(config));
      talons[i].setControl(new Follower(talons[0].getDeviceID(), MotorAlignmentValue.Aligned));
    }

    position = talons[0].getPosition();
    velocity = talons[0].getVelocity();
    appliedVolts = talons[0].getMotorVoltage();
    supplyCurrent = talons[0].getSupplyCurrent();
    torqueCurrent = talons[0].getTorqueCurrent();
    temp = talons[0].getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVolts, supplyCurrent, torqueCurrent);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
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
  public void setPID(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
    Slot0Configs cfg = new Slot0Configs();
    cfg.kP = kP;
    cfg.kI = kI;
    cfg.kD = kD;
    cfg.kS = kS;
    cfg.kV = kV;
    cfg.kA = kA;
    tryUntilOk(5, () -> talons[0].getConfigurator().apply(cfg));
  }

  @Override
  public void applyOutputs(FlywheelIOOutputs outputs) {
    switch (outputs.mode) {
      case COAST -> talons[0].setControl(coastControl);
      case VELOCITY -> {
        for (int i = 1; i < talons.length; i++) {
          if (i < outputs.num_motors) {
            talons[i].setControl(
                new Follower(talons[0].getDeviceID(), MotorAlignmentValue.Aligned));
          } else {
            talons[i].setControl(coastControl);
          }
        }
        talons[0].setControl(
            velocityControl.withVelocity(Units.radiansToRotations(outputs.velocityRadsPerSec)));
      }
      case VOLTAGE -> talons[0].setControl(voltageControl.withOutput(outputs.volts));
    }
  }
}
