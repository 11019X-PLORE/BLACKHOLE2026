package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTracer;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

public class LED extends FullSubsystem {
  // 硬件接口
  private final AddressableLED leds = new AddressableLED(LEDConstants.LEDPort);
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LEDConstants.length);

  // 状态枚举
  public enum LEDState {
    INITIAL, // 初始状态（联盟色）
    OFF, // 熄灭
    SHOOTING, // 正在发射（快速绿闪）
    READY_TO_SHOOT, // 瞄准完毕/转速达标（绿色常亮）
    TRENCH, // 处于隧道模式（黄色）
    INTAKING, // 正在吸球（白色流水）
    INTAKE_STOWED, // 进气臂收起
    OUTTAKE, // 正在传球（紫色闪烁）
    AUTO // 自动阶段（金光闪烁）
  }

  @Getter @Setter @AutoLogOutput private LEDState goal = LEDState.TRENCH;

  public LED() {
    leds.setLength(buffer.getLength());
    leds.setData(buffer);
    leds.start();
  }

  @Override
  public void periodic() {}

  @Override
  public void periodicAfterScheduler() {
    var alliance = DriverStation.getAlliance();
    Color allianceColor = Color.kWhite;
    if (alliance.isPresent()) {
      allianceColor = (alliance.get() == DriverStation.Alliance.Blue) ? Color.kBlue : Color.kRed;
    }

    switch (goal) {
      case INITIAL -> solidColor(allianceColor);
      case OFF -> solidColor(Color.kWhite);
      case SHOOTING -> strobe(Color.kGreen, Color.kBlack, 0.5); // 极快绿闪
      case READY_TO_SHOOT -> solidColor(Color.kGreen);
      case OUTTAKE -> wave(allianceColor, Color.kPurple, 10, 0.5);
      case TRENCH -> solidColor(Color.kYellow);
      case INTAKING -> wave(allianceColor, Color.kWhite, 10, 0.5);
      case INTAKE_STOWED -> wave(allianceColor, Color.kOrange, 10, 0.5);
      case AUTO -> rainbow(10, 0.5);
      default -> solidColor(Color.kWhite);
    }

    leds.setData(buffer);

    // 记录性能追踪
    LoggedTracer.record("LED/OutputData");
  }

  private void solidColor(Color color) {
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setLED(i, color);
    }
  }

  private void strobe(Color c1, Color c2, double duration) {
    boolean on = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
    solidColor(on ? c1 : c2);
  }

  private void rainbow(double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() / duration) % 1.0)) * 180.0;
    double xDiffPerLed = 180.0 / cycleLength;
    for (int i = 0; i < buffer.getLength(); i++) {
      x += xDiffPerLed;
      x %= 180.0;
      buffer.setHSV(i, (int) x, 255, 255);
    }
  }

  private void wave(Color c1, Color c2, double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
    for (int i = 0; i < buffer.getLength(); i++) {
      x += xDiffPerLed;
      double ratio = (Math.sin(x) + 1.0) / 2.0;
      double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
      double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
      double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
      buffer.setLED(i, new Color(red, green, blue));
    }
  }
}
