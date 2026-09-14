package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.geometry.AllianceFlipUtil;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  private static final double DEADBAND = 0.01;
  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  // Non-profiled heading controller used by aimAtTarget (feedforward supplies the velocity term).
  private static final double AIM_ANGLE_KP = 8.0;
  private static final double AIM_ANGLE_KD = 0.1;
  // Motion constraints for the profiled aim heading controller (rad/s, rad/s^2).
  private static final double AIM_ANGLE_MAX_VELOCITY = 8.0;
  private static final double AIM_ANGLE_MAX_ACCELERATION = 20.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  // Manual (teleop) speed percent limits for the demo dashboard. Preferences persist on the
  // roboRIO across reboots.
  private static final String DRIVE_SPEED_PERCENT_KEY = "Drive Speed Percent";
  private static final String TURN_SPEED_PERCENT_KEY = "Turn Speed Percent";

  static {
    // initDouble only writes the default when the key has never been saved.
    Preferences.initDouble(DRIVE_SPEED_PERCENT_KEY, 100.0);
    Preferences.initDouble(TURN_SPEED_PERCENT_KEY, 100.0);
    SmartDashboard.putData(
        "Speed Reset 100%",
        Commands.runOnce(
                () -> {
                  Preferences.setDouble(DRIVE_SPEED_PERCENT_KEY, 100.0);
                  Preferences.setDouble(TURN_SPEED_PERCENT_KEY, 100.0);
                })
            .ignoringDisable(true)
            .withName("SpeedReset100"));
  }

  private DriveCommands() {}

  /** Scale (0..1) for manual translational speed, read live from Preferences. */
  private static double getManualSpeedScale() {
    return MathUtil.clamp(Preferences.getDouble(DRIVE_SPEED_PERCENT_KEY, 100.0), 0.0, 100.0)
        / 100.0;
  }

  /** Scale (0..1) for manual turning speed, read live from Preferences. */
  private static double getTurnSpeedScale() {
    return MathUtil.clamp(Preferences.getDouble(TURN_SPEED_PERCENT_KEY, 100.0), 0.0, 100.0) / 100.0;
  }

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    // linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          // omega = Math.copySign(omega * omega, omega);
          Logger.recordOutput("Drive/Joystick/OmegaInput", omega);

          // Manual speed scaling from Preferences (demo limit)
          double driveScale = getManualSpeedScale();
          double turnScale = getTurnSpeedScale();
          Logger.recordOutput("Drive/SpeedScale", driveScale);
          Logger.recordOutput("Drive/TurnSpeedScale", turnScale);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec() * driveScale,
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec() * driveScale,
                  omega * drive.getMaxAngularSpeedRadPerSec() * turnScale);
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  AllianceFlipUtil.shouldFlip()
                      ? AllianceFlipUtil.apply(drive.getRotation())
                      : drive.getRotation()));
        },
        drive);
  }

  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));

    angleController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
            () -> {
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              double currentWrappedAngle = MathUtil.angleModulus(drive.getRotation().getRadians());
              double targetWrappedAngle =
                  MathUtil.angleModulus(rotationSupplier.get().getRadians());

              // Calculate angular speed using the wrapped angles
              double omega = angleController.calculate(currentWrappedAngle, targetWrappedAngle);

              // Manual speed scaling from Preferences (demo limit)
              double driveScale = getManualSpeedScale();
              double turnScale = getTurnSpeedScale();

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec() * driveScale,
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec() * driveScale,
                      omega * turnScale);
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      AllianceFlipUtil.shouldFlip()
                          ? AllianceFlipUtil.apply(drive.getRotation())
                          : drive.getRotation()));
            },
            drive)
        .beforeStarting(
            () -> {
              double currentWrappedAngle = MathUtil.angleModulus(drive.getRotation().getRadians());
              angleController.reset(currentWrappedAngle);
            });
  }

  /**
   * Aim-at-target drive command with a profiled heading PID <b>and</b> a yaw-velocity feedforward.
   *
   * <p>The driver keeps full translational control (field relative), while the chassis yaw is
   * servoed to a supplied field-relative heading. The heading controller is a {@link
   * ProfiledPIDController} so its slew is bounded by {@link #AIM_ANGLE_MAX_VELOCITY} and {@link
   * #AIM_ANGLE_MAX_ACCELERATION}. A feedforward angular velocity (e.g. the {@code
   * yawVelocityRadsPerSec} from a {@link frc.robot.util.Geoffrey.ShooterSetpoint}) is added on top
   * of the PID output so the robot leads a moving-shot instead of always lagging behind it.
   *
   * @param drive the drivetrain
   * @param xSupplier driver forward/back joystick (field relative, driver perspective)
   * @param ySupplier driver left/right joystick (field relative, driver perspective)
   * @param headingSupplier desired absolute field heading to face
   * @param feedforwardOmegaSupplier feedforward yaw rate (rad/s) added to the PID output
   */
  public static Command aimAtTarget(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> headingSupplier,
      DoubleSupplier feedforwardOmegaSupplier) {

    ProfiledPIDController headingController =
        new ProfiledPIDController(
            AIM_ANGLE_KP,
            0.001,
            AIM_ANGLE_KD,
            new TrapezoidProfile.Constraints(AIM_ANGLE_MAX_VELOCITY, AIM_ANGLE_MAX_ACCELERATION));
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
            () -> {
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              double measured = MathUtil.angleModulus(drive.getRotation().getRadians());
              double target = MathUtil.angleModulus(headingSupplier.get().getRadians());

              double pidOmega = headingController.calculate(measured, target);
              double ffOmega = feedforwardOmegaSupplier.getAsDouble();
              double omega = pidOmega + ffOmega;

              Logger.recordOutput("Drive/Aim/TargetHeadingRad", target);
              Logger.recordOutput("Drive/Aim/MeasuredHeadingRad", measured);
              Logger.recordOutput(
                  "Drive/Aim/SetpointHeadingRad", headingController.getSetpoint().position);
              Logger.recordOutput(
                  "Drive/Aim/SetpointVelocity", headingController.getSetpoint().velocity);
              Logger.recordOutput("Drive/Aim/FeedforwardOmega", ffOmega);
              Logger.recordOutput("Drive/Aim/CommandedOmega", omega);

              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      AllianceFlipUtil.shouldFlip()
                          ? AllianceFlipUtil.apply(drive.getRotation())
                          : drive.getRotation()));
            },
            drive)
        .beforeStarting(
            () ->
                headingController.reset(
                    MathUtil.angleModulus(drive.getRotation().getRadians()),
                    drive.getChassisSpeeds().omegaRadiansPerSecond));
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  // 有头模式
  public static Command joystickDriveRobotRelative(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {

    return Commands.run(
        () -> {
          // 获取摇杆线速度
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // 获取摇杆旋转速度
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);
          omega = Math.copySign(omega * omega, omega); // 平滑旋转

          // Manual speed scaling from Preferences (demo limit)
          double driveScale = getManualSpeedScale();
          double turnScale = getTurnSpeedScale();
          Logger.recordOutput("Drive/SpeedScale", driveScale);
          Logger.recordOutput("Drive/TurnSpeedScale", turnScale);

          // 转换为机体速度
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec() * driveScale,
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec() * driveScale,
                  omega * drive.getMaxAngularSpeedRadPerSec() * turnScale);

          drive.runVelocity(speeds);
        },
        drive);
  }

  public static Command joystickDriveFacingPoint(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      // DoubleSupplier omegaSupplier,
      Supplier<Translation2d> pointSupplier) {
    ProfiledPIDController headingController =
        new ProfiledPIDController(
            3.0,
            0.0,
            0.1,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // ===== 计算“朝向目标点”的角度 =====
          Translation2d robotPos = drive.getPose().getTranslation();
          Translation2d targetPoint = pointSupplier.get();

          Rotation2d targetAngle = targetPoint.minus(robotPos).getAngle();

          // Manual speed scaling from Preferences (demo limit)
          double driveScale = getManualSpeedScale();
          double omega =
              headingController.calculate(
                      drive.getRotation().getRadians(), targetAngle.getRadians())
                  * getTurnSpeedScale();

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec() * driveScale,
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec() * driveScale,
                  omega * drive.getMaxAngularSpeedRadPerSec());
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  AllianceFlipUtil.shouldFlip()
                      ? AllianceFlipUtil.apply(drive.getRotation())
                      : drive.getRotation()));
        },
        drive);
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }
}
