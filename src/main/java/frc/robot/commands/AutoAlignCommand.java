package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot.APResult;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoPilotConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.geometry.AllianceFlipUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AutoAlignCommand extends Command {

  Drive m_Drive;
  PIDController m_HeadingController = new PIDController(3.0, 0, 0.1);
  private boolean m_IsAtTarget = false;
  private Pose2d m_ActiveTarget; // 用于执行的最终坐标
  private double entryAngle;
  private double m_entryAngle;
  private Supplier<Pose2d> m_TargetPoseSupplier;

  public AutoAlignCommand(Drive drive, Supplier<Pose2d> _targetPoseSupplier, double _entryAngle) {
    addRequirements(drive);
    m_Drive = drive;
    m_TargetPoseSupplier = _targetPoseSupplier;
    m_HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    this.entryAngle = _entryAngle;
  }

  @Override
  public void initialize() {
    m_HeadingController.reset();
    m_IsAtTarget = false;

    Pose2d calculatedTargetPose = m_TargetPoseSupplier.get();
    m_ActiveTarget = calculatedTargetPose;

    if (AllianceFlipUtil.shouldFlip()) {
      m_entryAngle = 180.0 - entryAngle; // 翻转红方的入口角法则 (根据场地图可能需要微调)
    } else {
      m_entryAngle = entryAngle;
    }
  }

  @Override
  public void execute() {
    moveToTarget(m_ActiveTarget);
  }

  void moveToTarget(Pose2d _targetPose) {

    ChassisSpeeds m_RobotChassisSpeeds = m_Drive.getChassisSpeeds();
    APResult output =
        AutoPilotConstants.kAutopilot.calculate(
            m_Drive.getPose(),
            m_RobotChassisSpeeds,
            new APTarget(_targetPose).withEntryAngle(Rotation2d.fromDegrees(m_entryAngle))); // 要改
    m_IsAtTarget =
        AutoPilotConstants.kAutopilot.atTarget(m_Drive.getPose(), new APTarget(_targetPose));
    LinearVelocity veloX = output.vx();
    LinearVelocity veloY = output.vy();
    Rotation2d headingReference = output.targetAngle();
    double headingSpeed =
        m_HeadingController.calculate(
            m_Drive.getPose().getRotation().getRadians(), headingReference.getRadians());
    headingSpeed =
        MathUtil.clamp(
            headingSpeed,
            -AutoPilotConstants.kMaxAngularVelocity.in(Degrees),
            AutoPilotConstants.kMaxAngularVelocity.in(Degrees));
    // Logger.recordOutput("AutoAlignTestCode/VeloX", veloX.in(MetersPerSecond));

    Logger.recordOutput("AutoAlignTestCode/IsAtTarget", m_IsAtTarget);
    if (!m_IsAtTarget)
      m_Drive.runVelocity(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              veloX.in(MetersPerSecond),
              veloY.in(MetersPerSecond),
              headingSpeed,
              m_Drive.getPose().getRotation()));
    else {
      m_Drive.stop();
    }
  }

  @Override
  public void end(boolean _interrupted) {
    m_Drive.stop();
  }

  @Override
  public boolean isFinished() {
    return m_IsAtTarget;
  }
}
