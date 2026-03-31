package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.*;
import frc.robot.util.FuelSim;

public class TurretIOSim implements TurretIO {
  private final DCMotor gearbox = DCMotor.getKrakenX44Foc(1);
  private final DCMotorSim sim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.001, 100.0), gearbox);
  private final PIDController controller = new PIDController(0.003, 0, 0, Constants.loopPeriodSecs);
  // TODO ProfiledPIDController
  private double currentOutput = 0.0;
  private double appliedVoltage = 0.0;
  private boolean currentControl = false;
  // Fuel capacity
  private static final int CAPACITY = 10; // 你可以改成实际容量
  // 当前储存的 fuel 数量
  private int fuelStored = 100000;
  double turretHeight = 0.8;
  public Drive drive;
  private final FuelSim fuelSim;

  public TurretIOSim(FuelSim fuelSim) {
    this.fuelSim = fuelSim;
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    if (currentControl) {
      appliedVoltage = gearbox.getVoltage(currentOutput, sim.getAngularVelocityRadPerSec());
    }
    sim.setInputVoltage(MathUtil.clamp(appliedVoltage, -12.0, 12.0));
    sim.update(Constants.loopPeriodSecs);

    inputs.turretMotorConnected = true;
    inputs.positionRads = sim.getAngularPositionRad();
    inputs.velocityRadsPerSec = sim.getAngularVelocityRadPerSec();
    inputs.turretAppliedVolts = appliedVoltage;
    inputs.turretSupplyCurrent = sim.getCurrentDrawAmps();
    inputs.turretTorqueCurrent = currentOutput;
  }

  @Override
  public void setPID(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
    controller.setP(kP);
    controller.setD(kD);
  }

  public static final Transform2d ROBOT_TO_TURRET_TRANSFORM =
      new Transform2d(
          new Translation2d(0.3, 0.0), // turret 比机器人中心前方 0.3m，y轴居中
          new Rotation2d(0.0) // turret 初始朝向和机器人一样
          );

  @Override
  public void applyOutputs(TurretIOOutputs outputs) {
    switch (outputs.mode) {
      case BRAKE -> {
        currentControl = false;
        appliedVoltage = 0.0;
      }
      case COAST -> {
        currentControl = true;
        currentOutput = 0.0;
      }
      case CLOSED_LOOP -> {
        currentControl = true;
        currentOutput = controller.calculate(sim.getAngularPositionRad(), outputs.positionRads);
      }
      case POSITION_FOC -> {
        currentControl = true;
        currentOutput = controller.calculate(sim.getAngularPositionRad(), outputs.positionRads);
      }
    }
  }

  public boolean canIntake() {
    return fuelStored < CAPACITY;
  }

  public void intakeFuel() {
    fuelStored++;
  }

  // called repeatedly
  // TurretIOSim.java
  public void launchFuel() {
    if (fuelStored == 0) return;

    // 1. 获取射击参数
    ShotCalculator.ShootingParameters params = ShotCalculator.getInstance().getParameters();
    if (params == null) return; // 必须判空

    // 2. 燃料库存减少
    fuelStored--;

    // 3. 物理量转换
    // 假设你的飞轮半径是 0.05m (2英寸左右)，线速度 = 角速度 * 半径
    // 注意：params.flywheelSpeed() 的单位需确认是 Rad/s 还是 RPS
    double linearVelMps = params.flywheelSpeed() * 0.03;

    // 4. 调用发射
    // 注意：TurretAngle 应该是 Field-Relative (场地相对角度)
    fuelSim.launchFuel(
        MetersPerSecond.of(linearVelMps), // 初速度
        Radians.of(params.hoodAngle() + 0.6), // 俯仰角 (仰角)
        // Radians.of(0), // 水平朝向 (Rotation2d)
        Radians.of(params.turretAngle().getRadians()), // 水平朝向 (Rotation2d)
        Meters.of(TurretConstants.turret_height_meters) // 发射高度
        );
  }
}
