package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FieldConstants.AprilTagLayoutType;
import frc.robot.autos.LeftAuto;
import frc.robot.autos.MidAuto;
import frc.robot.autos.MidAutoShort;
import frc.robot.autos.RightAuto;
import frc.robot.autos.RightCycleAuto;
import frc.robot.autos.Test;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.extension.Extension;
import frc.robot.subsystems.extension.ExtensionConstants;
import frc.robot.subsystems.extension.ExtensionIOReal;
import frc.robot.subsystems.extension.ExtensionIOSim;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOReal;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOReal;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodConstants;
import frc.robot.subsystems.shooter.hood.HoodIOReal;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.TurretConstants;
import frc.robot.subsystems.shooter.turret.TurretIOSim;
import frc.robot.subsystems.shooter.turret.TurretIOreal;
import frc.robot.subsystems.superstructure.SuperstructureFactory;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.ClimbTargetSelector;
import frc.robot.util.Dimensions;
import frc.robot.util.FuelSim;
import frc.robot.util.TrenchHelper;
import frc.robot.util.geometry.AllianceFlipUtil;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  // Subsystems
  public final Drive drive;
  public final Intake intake;
  public final Extension extension;
  public final Indexer indexer;
  public final Turret turret;
  public final Flywheel flywheel;
  public final Hood hood;
  public final Vision vision;
  public final LED led;

  public Dimensions dimensions;
  public FuelSim fuelSim = new FuelSim("FuelSim"); // creates a new fuelSim of FuelSim
  // Controller
  private final CommandPS5Controller controller = new CommandPS5Controller(0);

  private final CommandPS5Controller testing_controller = new CommandPS5Controller(1);

  // Bindings
  private final Trigger zeroSuperstructurePosition = controller.square();
  private final Trigger zeroGyro = controller.button(13);
  private final Trigger intakeTrigger = controller.R1();
  private final Trigger outtakeTrigger = controller.L2();
  private final Trigger shakeStowTrigger = controller.circle();
  private final Trigger stowTrigger = controller.button(10);
  private final Trigger shootTrigger = controller.R2();
  private final Trigger passTrigger = controller.L1();
  private final Trigger fixShootTrigger = controller.triangle();
  private final Trigger driveToClimb = controller.povLeft();
  private final Trigger autoTrench = controller.povRight();
  private final Trigger drivetoBump = controller.cross();
  private final Trigger robotHeadSwitchTrigger = controller.button(12);
  private final Trigger driveFaceToPointTrigger = controller.button(11);
  private final Trigger shootouttakTrigger = controller.button(9);

  private final Alert controllerDisconnected =
      new Alert("controller disconnected (port 0).", AlertType.kWarning);
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final Timer simShootTimer = new Timer();
  private boolean isTestMode = false;

  public RobotContainer() {
    simShootTimer.start();
    switch (Constants.currentMode) {
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        intake =
            new Intake(
                new IntakeIOReal(IntakeConstants.kIntakeId, IntakeConstants.kIntakeInverted));
        indexer = new Indexer(new IndexerIOReal());
        extension =
            new Extension(
                new ExtensionIOReal(
                    ExtensionConstants.kExtensionId, ExtensionConstants.kExtensionInverted));
        turret =
            new Turret(
                new TurretIOreal(TurretConstants.kTurretID, TurretConstants.kTurretInverted),
                TurretConstants.kTurretonRobotoffset,
                drive::getPose,
                drive::getFieldVelocity);
        TurretConstants.swerve2TurretStructure.setBase(drive);
        turret.setBase(TurretConstants.swerve2TurretStructure);

        vision =
            new Vision(
                drive,
                // new VisionIOLimelight(
                //     camera0Name,
                //     new double[] {640, 480},
                //     drive,
                //     new Transform3d(0.0, 0.0, 0.616, new Rotation3d(0.0, -0.4, 0.0))),
                new VisionIOLimelight(
                    camera1Name,
                    new double[] {640, 480},
                    turret,
                    TurretConstants.kCameraonTurretoffset));
        flywheel =
            new Flywheel(
                new FlywheelIOReal(
                    FlywheelConstants.kFlywheelId, FlywheelConstants.kFlywheelInverted),
                turret);
        hood = new Hood(new HoodIOReal(HoodConstants.kHoodId, HoodConstants.kHoodInverted), turret);
        led = new LED();
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision =
            new Vision(
                drive,
                new VisionIOPhotonVisionSim(
                    camera0Name, robotToCamera0, drive, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(
                    camera1Name, robotToCamera1, drive, robotToCamera1, drive::getPose));
        intake = new Intake(new IntakeIOSim());
        indexer = new Indexer(new IndexerIOSim());
        extension = new Extension(new ExtensionIOSim());
        turret =
            new Turret(
                new TurretIOSim(fuelSim),
                TurretConstants.kTurretonRobotoffset,
                drive::getPose,
                drive::getFieldVelocity);
        TurretConstants.swerve2TurretStructure.setBase(drive);
        turret.setBase(TurretConstants.swerve2TurretStructure);
        flywheel = new Flywheel(new FlywheelIOSim(), turret);
        // upperStructure = new UpperStructure(turret, turretFR);
        hood = new Hood(new HoodIOSim(), turret);
        led = new LED();
        // fuel sim setup
        fuelSim.spawnStartingFuel();
        // 2. 调用你写的配置方法 (确保传入 intake 的状态判定)
        configureFuelSimRobot(
            () -> intake.getGoal() == Intake.IntakeGoal.INTAKE,
            () -> {
              System.out.println("yesyesyes Fuel");
            } // 这里可以写 intake 成功后的回调
            );
        // 物理参数设置
        fuelSim.setSubticks(1); // 每 20ms 执行多少次物理迭代
        fuelSim.enableAirResistance(); // 开启空气阻力影响（可选）

        // 控制模拟是否自动运行
        fuelSim.start(); // 启动模拟
        // … 在适当的时候 …
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision =
            new Vision(
                drive,
                new VisionIOPhotonVisionSim(
                    camera0Name, robotToCamera0, drive, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(
                    camera1Name, robotToCamera1, drive, robotToCamera1, drive::getPose));
        intake = new Intake(new IntakeIOSim());
        indexer = new Indexer(new IndexerIOSim());
        extension = new Extension(new ExtensionIOSim());
        turret =
            new Turret(
                new TurretIOSim(fuelSim),
                TurretConstants.kTurretonRobotoffset,
                drive::getPose,
                drive::getFieldVelocity);
        TurretConstants.swerve2TurretStructure.setBase(drive);
        turret.setBase(TurretConstants.swerve2TurretStructure);
        flywheel = new Flywheel(new FlywheelIOSim(), turret);
        hood = new Hood(new HoodIOSim(), turret);
        led = new LED();
        break;
    }
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser.addOption("LEFT", new LeftAuto(this));
    autoChooser.addOption("MID", new MidAuto(this));
    autoChooser.addOption("MIDSHORT", new MidAutoShort(this));
    autoChooser.addOption("RIGHT", new RightAuto(this));
    autoChooser.addOption("RIGHTCYCLE", new RightCycleAuto(this));
    autoChooser.addOption("TEST", new Test(this));
    autoChooser.addDefaultOption("LEFT", new LeftAuto(this));
    // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    configureButtonBindings();
  }

  public Turret getTurret() {
    return turret;
  }

  public Hood getHood() {
    return hood;
  }

  public Flywheel getFlywheel() {
    return flywheel;
  }

  public Intake getIntake() {
    return intake;
  }

  public Extension getExtension() {
    return extension;
  }

  public Indexer getIndexer() {
    return indexer;
  }

  public LED getLED() {
    return led;
  }

  public Drive getDrive() {
    return drive;
  }

  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // zero gyro
    zeroGyro.onTrue(Commands.runOnce(drive::zeroHeading, drive).ignoringDisable(true));

    // bump
    drivetoBump.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> Rotation2d.fromDegrees(45)));

    // 有头模式
    robotHeadSwitchTrigger.whileTrue(
        DriveCommands.joystickDriveRobotRelative(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    driveFaceToPointTrigger.whileTrue(
        DriveCommands.joystickDriveFacingPoint(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d())));

    zeroSuperstructurePosition.onTrue(
        (Commands.parallel(hood.zeroCommand(), extension.zeroCommand(), turret.zeroCommand())));

    fixShootTrigger.onTrue(
        Commands.runOnce(
            () -> {
              if (isTestMode) {
                SuperstructureFactory.idle(this).schedule();
                isTestMode = false;
              } else {
                SuperstructureFactory.test(this).schedule();
                isTestMode = true;
              }
            }));

    shootTrigger
        .onTrue(SuperstructureFactory.shoot(this))
        .onFalse(SuperstructureFactory.activeShooting(this));
    passTrigger
        .onTrue(SuperstructureFactory.pass(this))
        .onFalse(SuperstructureFactory.activeShooting(this));
    shootTrigger
        .or(passTrigger)
        .onTrue(
            Commands.parallel(
                SuperstructureFactory.feeding(this),
                intake.setGoalCommand(Intake.IntakeGoal.SHOOT)))
        .onFalse(
            new ConditionalCommand(
                Commands.parallel(
                    SuperstructureFactory.runIndexerIntake(this),
                    intake.setGoalCommand(Intake.IntakeGoal.INTAKE)),
                Commands.parallel(
                    SuperstructureFactory.stopFeeding(this),
                    intake.setGoalCommand(Intake.IntakeGoal.STOP)),
                intakeTrigger));

    intakeTrigger
        .onTrue(
            Commands.parallel(
                intake.setGoalCommand(Intake.IntakeGoal.INTAKE),
                extension.setGoalCommand(Extension.ExtensionGoal.DEPLOYED),
                new ConditionalCommand(
                    SuperstructureFactory.feeding(this),
                    SuperstructureFactory.runIndexerIntake(this),
                    shootTrigger.or(passTrigger))))
        .onFalse(
            Commands.parallel(
                extension.setGoalCommand(Extension.ExtensionGoal.FEEDING),
                new ConditionalCommand(
                    Commands.parallel(
                        SuperstructureFactory.feeding(this),
                        intake.setGoalCommand(Intake.IntakeGoal.SHOOT)),
                    Commands.parallel(
                        SuperstructureFactory.stopFeeding(this),
                        intake.setGoalCommand(Intake.IntakeGoal.STOP)),
                    shootTrigger.or(passTrigger))));

    outtakeTrigger
        .onTrue(
            Commands.parallel(
                intake.setGoalCommand(Intake.IntakeGoal.OUTTAKE),
                extension.setGoalCommand(Extension.ExtensionGoal.DEPLOYED),
                SuperstructureFactory.spit(this)))
        .onFalse(
            Commands.parallel(
                intake.setGoalCommand(Intake.IntakeGoal.STOP),
                extension.setGoalCommand(Extension.ExtensionGoal.DEPLOYED),
                SuperstructureFactory.stopFeeding(this)));

    shakeStowTrigger
        .onTrue(
            Commands.parallel(
                intake.setGoalCommand(Intake.IntakeGoal.STOW),
                extension.setGoalCommand(Extension.ExtensionGoal.SHAKE)))
        .onFalse(
            Commands.parallel(
                intake.setGoalCommand(Intake.IntakeGoal.STOP),
                extension.setGoalCommand(Extension.ExtensionGoal.STOWED)));

    stowTrigger.onTrue(
        Commands.parallel(
            intake.setGoalCommand(Intake.IntakeGoal.STOP),
            extension.setGoalCommand(Extension.ExtensionGoal.STOWED)));

    shootouttakTrigger
        .onTrue(SuperstructureFactory.shootSpit(this))
        .onFalse(SuperstructureFactory.activeShooting(this));

    driveToClimb.whileTrue(
        new AutoAlignCommand(
            drive,
            () -> ClimbTargetSelector.getNearestClimbPose(drive::getPose),
            ClimbTargetSelector.getNearestClimbPose(drive::getPose).getRotation().getDegrees()));

    autoTrench.whileTrue(
        Commands.parallel(
            SuperstructureFactory.trench(this),
            Commands.defer(
                () ->
                    DriveCommands.autoPathfindToPose(
                        drive, TrenchHelper.getTrenchTargetPose(drive::getPose)),
                java.util.Set.of(drive) // 声明占用 drive 子系统
                )));

    // testing:
    testing_controller
        .button(1)
        .onTrue(
            Commands.parallel(
                flywheel.setGoalCommand(Flywheel.FlywheelGoal.FIXED_VELOCITY),
                indexer.setGoalCommand(Indexer.IndexerGoal.SHOOT)))
        .onFalse(
            Commands.parallel(
                flywheel.setGoalCommand(Flywheel.FlywheelGoal.IDLE),
                indexer.setGoalCommand(Indexer.IndexerGoal.STOP)));
  }

  private void configureFuelSim() {
    fuelSim = new FuelSim();
    fuelSim.spawnStartingFuel();

    fuelSim.start();
    SmartDashboard.putData(
        Commands.runOnce(
                () -> {
                  fuelSim.clearFuel();
                  fuelSim.spawnStartingFuel();
                })
            .withName("Reset Fuel")
            .ignoringDisable(true));
  }

  private void configureFuelSimRobot(BooleanSupplier ableToIntake, Runnable intakeCallback) {
    // 注册机器人实体 (1.0m x 1.0m)
    fuelSim.registerRobot(
        Dimensions.FULL_WIDTH,
        Dimensions.FULL_LENGTH,
        Dimensions.BUMPER_HEIGHT,
        // turret::getturretpose,
        drive::getPose,
        drive::getFieldVelocity);

    // 重新定义吸入口坐标：
    // 假设 X 正方向是机器人的前方
    double frontEdge = Dimensions.FULL_LENGTH / 2.0;
    double intakeDepth = 0.4; // 吸入口向外延伸 20cm
    double intakeWidth = Dimensions.FULL_WIDTH * 1.8; // 覆盖 80% 的车宽

    fuelSim.registerIntake(
        frontEdge, // xMin: 刚好从前保险杠开始
        frontEdge + intakeDepth, // xMax: 向前延伸出保险杠
        -intakeWidth / 2.0, // yMin: 中心对称
        intakeWidth / 2.0, // yMax: 中心对称
        () -> intake.getGoal() == Intake.IntakeGoal.INTAKE, // 调试用：先强制设为 true 看看能不能吸到
        intakeCallback);
  }

  /** 供 Robot.java 在仿真模式下周期性调用 */

  /** Update dashboard outputs. */
  public void updateDashboardOutputs() {
    // Publish match time
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

    SmartDashboard.putNumber("Batter Voltage", RobotController.getBatteryVoltage());
    // Controller disconnected alerts
    controllerDisconnected.set(!DriverStation.isJoystickConnected(controller.getHID().getPort()));

    // secondaryDisconnected.set(!DriverStation.isJoystickConnected(secondary.getHID().getPort()));
    // overrideDisconnected.set(!overrides.isConnected());
  }

  private void handleSimulationShooting() {
    // 只有在模拟环境下才执行后续计算，节省真实机器人的 CPU 资源
    if (!Robot.isSimulation()) {
      return;
    }

    // 检查所有射击机构是否到位
    boolean isReadyToShoot = flywheel.atGoal() && hood.atGoal() && turret.atGoal();

    // 记录状态供 Dashboard/AdvantageScope 查看
    Logger.recordOutput("Superstructure/ReadyToShoot", isReadyToShoot);

    // 执行发射判定 - 当所有机构就位时发射
    if (isReadyToShoot) {

      // 严格控制射频：每 0.25 秒最多只能发射一颗球
      if (simShootTimer.hasElapsed(0.25)) {
        simShootTimer.restart();

        // 计算出球线速度 (v = ω * r)
        double linearVelMetersPerSec = flywheel.getVelocity() * FlywheelConstants.kFlywheelRadius;

        // 调用模拟器发射
        fuelSim.launchFuel(
            MetersPerSecond.of(linearVelMetersPerSec),
            Radians.of(hood.getMeasuredAngleRad()),
            // 这里使用炮塔相对于底盘的角度
            Radians.of(turret.getRobotToTurret().getAngle().getRadians()),
            Meters.of(0.6) // 假设射出口高度为 0.6 米
            );

        Logger.recordOutput("Superstructure/SimLastShotTime", Timer.getFPGATimestamp());
      }
    }
  }

  /** Returns the current AprilTag layout type. */
  public AprilTagLayoutType getSelectedAprilTagLayout() {
    return FieldConstants.defaultAprilTagType;
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
