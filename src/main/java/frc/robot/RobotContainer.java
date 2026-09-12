package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.CommandFactories.CommandFactory;
import frc.robot.FieldConstants.AprilTagLayoutType;
import frc.robot.autos.FCeStR;
import frc.robot.autos.LCPCeP;
import frc.robot.autos.LCSR;
import frc.robot.autos.LCePCeR;
import frc.robot.autos.LCeSR;
import frc.robot.autos.LCsSCsS;
import frc.robot.autos.MSm;
import frc.robot.autos.MSmR;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmGoal;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmIOReal;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.blocker.Blocker;
import frc.robot.subsystems.blocker.Blocker.BlockerGoal;
import frc.robot.subsystems.blocker.BlockerConstants;
import frc.robot.subsystems.blocker.BlockerIOReal;
import frc.robot.subsystems.blocker.BlockerIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelConstants;
import frc.robot.subsystems.flywheel.FlywheelIOReal;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.indexer.IndexerIOReal;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.Dimensions;
import frc.robot.util.FuelSim;
import frc.robot.util.Geoffrey.PhysicalJoint;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  // --- Subsystems ---
  public final Drive drive;
  public final Vision vision;
  public final Indexer indexer;
  public final Flywheel flywheel;
  public final Arm arm;
  public final Blocker blocker;
  public final LED led;

  // Simulated fuel (balls) used for visualisation in sim.
  public FuelSim fuelSim = new FuelSim("FuelSim");

  // --- Controller ---
  private final CommandPS5Controller controller = new CommandPS5Controller(0);

  // --- Bindings ---
  private final Trigger zeroGyro = controller.button(13);
  private final Trigger intakeTrigger = controller.R1();
  private final Trigger deployIntakeTrigger = controller.button(1);
  private final Trigger shootTrigger = controller.R2();
  private final Trigger autoShootTrigger = controller.circle();
  private final Trigger outtakeTrigger = controller.button(2);
  //   private final Trigger deployArmTrigger = controller.triangle();
  private final Trigger passingTrigger = controller.L2();

  //   private final Trigger stowArmTrigger = controller.cross();
  private final Trigger blockTrigger = controller.L1();
  private final Trigger bigbackTrigger = controller.triangle();

  private final Trigger increaseTurretOffset = controller.povUp();
  private final Trigger decreaseTurretOffset = controller.povDown();

  private final Trigger increaseBlockerOffset = controller.povRight();
  private final Trigger decreaseBlockerOffset = controller.povLeft();

  private final Trigger testShootTrigger = controller.button(10);

  private final Alert controllerDisconnected =
      new Alert("controller disconnected (port 0).", AlertType.kWarning);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardChooser<Boolean> sideChooser;

  @AutoLogOutput private boolean isbigback = false;
  @AutoLogOutput private boolean isIntaking = false;
  @AutoLogOutput private boolean isAutoAim = true;

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL -> {
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        // Two cameras mounted on the chassis (drive).
        vision =
            new Vision(
                drive,
                new VisionIOLimelight(camera0Name, new double[] {640, 480}, drive, robotToCamera0),
                new VisionIOLimelight(camera1Name, new double[] {640, 480}, drive, robotToCamera1));
        arm = new Arm(new ArmIOReal(ArmConstants.kArmId, ArmConstants.kArmInverted));

        indexer =
            new Indexer(
                new IndexerIOReal(
                    IndexerConstants.kIndexerId,
                    IndexerConstants.kIndexerInverted,
                    IndexerConstants.kTriggerId,
                    IndexerConstants.kTriggerInverted),
                arm);
        blocker =
            new Blocker(
                new BlockerIOReal(BlockerConstants.kBlockerId, BlockerConstants.kBlockerInverted),
                arm);
        flywheel =
            new Flywheel(
                new FlywheelIOReal(
                    FlywheelConstants.kFlywheelIds, FlywheelConstants.kFlywheelInverted),
                arm);
        led = new LED();
      }

      case SIM -> {
        // Physics sim IO implementations
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
        arm = new Arm(new ArmIOSim());
        indexer = new Indexer(new IndexerIOSim(), arm);
        blocker = new Blocker(new BlockerIOSim(), arm);
        flywheel = new Flywheel(new FlywheelIOSim(), arm);
        led = new LED();
        setupFuelSim();
      }

      default -> {
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
        arm = new Arm(new ArmIOSim());

        indexer = new Indexer(new IndexerIOSim(), arm);
        blocker = new Blocker(new BlockerIOSim(), arm);
        flywheel = new Flywheel(new FlywheelIOSim(), arm);
        led = new LED();
      }
    }

    // --- Geoffrey physical joint chain ---
    // The chassis (drive) is the base of the chain. The arm and blocker are mounted on the
    // chassis via static structure joints, and each rotates about its measured angle.
    PhysicalJoint armMount = PhysicalJoint.getStructureJoint(ArmConstants.kChassisToArmPivot);
    armMount.setBase(drive);
    arm.setBase(armMount);

    PhysicalJoint blockerMount =
        PhysicalJoint.getStructureJoint(BlockerConstants.kChassisToBlockerPivot);
    blockerMount.setBase(drive);
    blocker.setBase(blockerMount);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    sideChooser = new LoggedDashboardChooser<>("Side Chooser");
    sideChooser.addDefaultOption("Left (Default)", false); // 默认 false
    sideChooser.addOption("Right (Flipped)", true); // 选中时 true

    autoChooser.addOption("LCSR", new LCSR(this, sideChooser::get));
    autoChooser.addOption("LCeSR", new LCeSR(this, sideChooser::get));
    autoChooser.addOption("LCePCeR", new LCePCeR(this, sideChooser::get));
    autoChooser.addOption("LCPCeP", new LCPCeP(this, sideChooser::get));
    autoChooser.addOption("FCeStR", new FCeStR(this, sideChooser::get));
    autoChooser.addOption("LCsSCsS", new LCsSCsS(this, sideChooser::get));
    autoChooser.addOption("MSm", new MSm(this, sideChooser::get));
    autoChooser.addOption("MSmR", new MSmR(this, sideChooser::get));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // Default command: field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Zero the gyro heading
    zeroGyro.onTrue(Commands.runOnce(drive::zeroHeading, drive).ignoringDisable(true));

    // Intake: deploy arm, run flywheel + indexer inward
    intakeTrigger
        .whileTrue(
            Commands.parallel(
                // arm.setGoalCommand(Arm.ArmGoal.INTAKE),
                flywheel.setGoalCommand(Flywheel.FlywheelGoal.INTAKE),
                indexer.setGoalCommand(Indexer.IndexerGoal.INTAKE)))
        .onTrue(
            new InstantCommand(
                () -> {
                  isIntaking = true;
                  arm.setGoal(ArmGoal.INTAKE);

                  blocker.setGoal(BlockerGoal.CLOSE);
                },
                arm))
        .onFalse(
            Commands.parallel(
                // arm.setGoalCommand(Arm.ArmGoal.STOW),
                flywheel.setGoalCommand(Flywheel.FlywheelGoal.IDLE),
                indexer.setGoalCommand(Indexer.IndexerGoal.STOP)));

    deployIntakeTrigger.onTrue(
        new InstantCommand(
                () -> {
                  isIntaking = false;
                  arm.setGoal(ArmGoal.ZEROING);
                  //   flywheel.setGoal(isIntaking ? FlywheelGoal.INTAKE : FlywheelGoal.IDLE);
                  //   indexer.setGoal(isIntaking ? Indexer.IndexerGoal.INTAKE :
                  // Indexer.IndexerGoal.STOP);
                },
                arm)
            .andThen(blocker.setGoalCommand(BlockerGoal.CLOSE))
        // .andThen(
        //     new InstantCommand(
        //         () -> {
        //           isIntaking = false;
        //         }))
        );

    // Shoot: for now the fixed shooting fallback is the default while the shoot button is held
    // (used when vision is unavailable/unreliable). The arm holds its fixed shooting angle and the
    // flywheel spins to its fixed shoot velocity, feeding once both are at goal. Swap to
    // CommandFactory.shoot(...) below to re-enable the vision-aimed, shoot-on-the-move version.
    shootTrigger
        .whileTrue(Commands.parallel(CommandFactory.shootFixed(this), blocker.shootCommand()))
        .onFalse(
            blocker
                .setGoalCommand(BlockerGoal.OPEN)
                .andThen(new InstantCommand(() -> isbigback = true)));
    testShootTrigger
        .whileTrue(Commands.parallel(CommandFactory.shootTest(this), blocker.shootCommand()))
        .onFalse(
            blocker
                .setGoalCommand(BlockerGoal.OPEN)
                .andThen(new InstantCommand(() -> isbigback = true)));
    autoShootTrigger
        .whileTrue(
            Commands.parallel(
                CommandFactory.shoot(
                    this, () -> -controller.getLeftY(), () -> -controller.getLeftX()),
                blocker.shootCommand()))
        .onFalse(
            blocker
                .setGoalCommand(BlockerGoal.OPEN)
                .andThen(new InstantCommand(() -> isbigback = true)));
    shootTrigger
        .or(autoShootTrigger)
        .or(testShootTrigger)
        .onTrue(
            new InstantCommand(
                () -> {
                  isIntaking = false;
                }));

    // shootTrigger.whileTrue(
    //     CommandFactory.shoot(this, () -> -controller.getLeftY(), () -> -controller.getLeftX()));

    // Outtake: reverse flywheel + indexer
    outtakeTrigger
        .whileTrue(
            Commands.parallel(
                flywheel.setGoalCommand(Flywheel.FlywheelGoal.OUTTAKE),
                indexer.setGoalCommand(Indexer.IndexerGoal.OUTTAKE),
                new InstantCommand(
                    () -> {
                      isIntaking = false;
                    })))
        .onFalse(
            Commands.parallel(
                flywheel.setGoalCommand(Flywheel.FlywheelGoal.IDLE),
                indexer.setGoalCommand(Indexer.IndexerGoal.STOP)));

    passingTrigger
        .whileTrue(Commands.parallel(CommandFactory.passing(this), blocker.shootCommand()))
        .onFalse(
            blocker
                .setGoalCommand(BlockerGoal.HALFOPEN)
                .andThen(new InstantCommand(() -> isbigback = true)));
    // Manual arm control
    // deployArmTrigger.onTrue(arm.setGoalCommand(Arm.ArmGoal.INTAKE));
    // stowArmTrigger.onTrue(arm.setGoalCommand(Arm.ArmGoal.STOW));

    // Blocker: deploy while held, retract when released. Ensure we read isbigback at runtime
    // (the previous code evaluated the ternary at bind time, so it always used the initial value).
    blockTrigger
        .whileTrue(blocker.setGoalCommand(BlockerGoal.BLOCKING))
        .onFalse(
            Commands.runOnce(
                () -> blocker.setGoal(isbigback ? BlockerGoal.HALFOPEN : BlockerGoal.CLOSE),
                blocker));

    // Toggle the "big back" mode and then set the blocker goal based on the new value.
    bigbackTrigger.onTrue(
        new InstantCommand(
            () -> {
              isbigback = !isbigback;
              blocker.setGoal(isbigback ? BlockerGoal.HALFOPEN : BlockerGoal.CLOSE);
            },
            blocker));

    increaseTurretOffset.onTrue(
        new InstantCommand(
            () -> {
              ArmIOReal.angleOffset += Math.toRadians(2.5);
            }));
    decreaseTurretOffset.onTrue(
        new InstantCommand(
            () -> {
              ArmIOReal.angleOffset -= Math.toRadians(2.5);
            }));

    increaseBlockerOffset.onTrue(blocker.upZeroCommand());
    decreaseBlockerOffset.onTrue(blocker.downZeroCommand());
  }

  /** Minimal fuel (ball) simulation used only for visualisation in sim. */
  private void setupFuelSim() {
    fuelSim.spawnStartingFuel();
    fuelSim.registerRobot(
        Dimensions.FULL_WIDTH,
        Dimensions.FULL_LENGTH,
        Dimensions.BUMPER_HEIGHT,
        drive::getPose,
        drive::getFieldVelocity);

    // Intake zone in front of the robot, active while the flywheel is intaking.
    double frontEdge = Dimensions.FULL_LENGTH / 2.0;
    fuelSim.registerIntake(
        frontEdge,
        frontEdge + 0.4,
        -Dimensions.FULL_WIDTH,
        Dimensions.FULL_WIDTH,
        () -> flywheel.getGoal() == Flywheel.FlywheelGoal.INTAKE,
        () -> {});

    fuelSim.setSubticks(1);
    fuelSim.enableAirResistance();
    fuelSim.start();
  }

  public Drive getDrive() {
    return drive;
  }

  /** Returns the current AprilTag layout type. */
  public AprilTagLayoutType getSelectedAprilTagLayout() {
    return FieldConstants.defaultAprilTagType;
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
