package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

public class LED extends SubsystemBase {
  private AddressableLED leds = new AddressableLED(LEDConstants.LEDPort);
  private AddressableLEDBuffer buffer = new AddressableLEDBuffer(LEDConstants.length);

  public enum LEDState {
    INITIAL, // 初始状态（联盟色）
    OFF, // 熄灭
    SHOOTING, // 正在发射（快速闪烁）
    READY_TO_SHOOT, // 瞄准完毕/转速达标（绿色常亮）
    TRENCH, // 处于隧道模式（黄色）
    INTAKING, // 正在吸球
    INTAKE_STOWED, // 进气臂收起
    PASSING, // 正在传球
    CLIMBING, // 爬升中
    AUTO // 自动阶段
  }

  @Getter @Setter @AutoLogOutput private LEDState goal = LEDState.INITIAL;

  public LED() {
    leds.setLength(buffer.getLength());
    leds.setData(buffer);
    leds.start();
  }

  @Override
  public void periodic() {
    // 获取当前联盟颜色
    var alliance = DriverStation.getAlliance();
    Color allianceColor = Color.kBlack;
    if (alliance.isPresent()) {
      allianceColor = (alliance.get() == DriverStation.Alliance.Blue) ? Color.kBlue : Color.kRed;
    }

    // 根据目标状态执行灯效
    switch (goal) {
      case INITIAL -> solidColor(allianceColor);
      case OFF -> solidColor(Color.kBlack);
      case SHOOTING -> strobe(Color.kGreen, Color.kBlack, 0.5); // 快速绿闪
      case READY_TO_SHOOT -> solidColor(Color.kGreen); // 绿色提示可以开火
      case PASSING -> strobe(Color.kPurple, Color.kBlack, 0.5); // 快速紫闪
      case TRENCH -> solidColor(Color.kYellow); // 黄色代表安全高度
      case INTAKING -> wave(allianceColor, Color.kWhite, 10, 0.5);
      case INTAKE_STOWED -> wave(allianceColor, Color.kOrange, 10, 0.5);
      case CLIMBING -> rainbow(10, 0.5);
      case AUTO -> strobe(Color.kGold, Color.kBlack, 0.5);
      default -> solidColor(Color.kBlack);
    }

    leds.setData(buffer);
  }

  // --- 基础灯效方法 ---
  private void solidColor(Color color) {
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setLED(i, color);
    }
  }

  private void strobe(Color c1, Color c2, double duration) {
    boolean on = ((Timer.getFPGATimestamp() % (duration * 2e6)) / (duration * 2e6)) > 0.5;
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
