package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import frc.robot.generated.TunerConstants;
import java.util.Queue;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon =
      new Pigeon2(TunerConstants.DrivetrainConstants.Pigeon2Id, TunerConstants.kCANBus);
  private final StatusSignal<Angle> roll = pigeon.getRoll();
  private final StatusSignal<Angle> pitch = pigeon.getPitch();
  private final StatusSignal<Angle> yaw = pigeon.getYaw();

  private final Queue<Double> rollPositionQueue;
  private final Queue<Double> pitchPositionQueue;
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<AngularVelocity> yawVelocity;
  private final Queue<Double> yawVelocityQueue;
  private final StatusSignal<LinearAcceleration> accelX;
  private final StatusSignal<LinearAcceleration> accelY;
  private final StatusSignal<LinearAcceleration> accelZ;
  private final Queue<Double> accelXQueue;
  private final Queue<Double> accelYQueue;
  private final Queue<Double> accelZQueue;

  public GyroIOPigeon2() {
    if (TunerConstants.DrivetrainConstants.Pigeon2Configs != null) {
      pigeon.getConfigurator().apply(TunerConstants.DrivetrainConstants.Pigeon2Configs);
    } else {
      pigeon.getConfigurator().apply(new Pigeon2Configuration());
    }

    pigeon.getConfigurator().setYaw(0.0);
    yawVelocity = pigeon.getAngularVelocityZWorld();
    accelX = pigeon.getAccelerationX();
    accelY = pigeon.getAccelerationY();
    accelZ = pigeon.getAccelerationZ();

    roll.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY);
    pitch.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY);
    yaw.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY);
    yawVelocity.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY);
    accelX.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY);
    accelY.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY);
    accelZ.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY);
    pigeon.optimizeBusUtilization();
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    rollPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(roll.clone());
    pitchPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pitch.clone());
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(yaw.clone());
    yawVelocityQueue = PhoenixOdometryThread.getInstance().registerSignal(yawVelocity.clone());
    accelXQueue = PhoenixOdometryThread.getInstance().registerSignal(accelX.clone());
    accelYQueue = PhoenixOdometryThread.getInstance().registerSignal(accelY.clone());
    accelZQueue = PhoenixOdometryThread.getInstance().registerSignal(accelZ.clone());
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(roll, pitch, yaw, yawVelocity, accelX, accelY, accelZ)
            .equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    inputs.rotation =
        new Rotation3d(
            Units.degreesToRadians(pitch.getValueAsDouble()),
            Units.degreesToRadians(roll.getValueAsDouble()),
            inputs.yawPosition.getRadians());

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    inputs.odometryPitchPositions =
        pitchPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    inputs.odometryRollPositions =
        rollPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    inputs.odometryYawVelocities =
        yawVelocityQueue.stream()
            .mapToDouble((Double value) -> Units.degreesToRadians(value))
            .toArray();
    inputs.odometryAccelX = accelXQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryAccelY = accelYQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryAccelZ = accelZQueue.stream().mapToDouble((Double value) -> value).toArray();
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
    rollPositionQueue.clear();
    pitchPositionQueue.clear();
    yawVelocityQueue.clear();
    accelXQueue.clear();
    accelYQueue.clear();
    accelZQueue.clear();
  }
}
