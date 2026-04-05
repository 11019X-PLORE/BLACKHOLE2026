package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Ports {
  // CAN Buses
  public static final CANBus kRoboRioCANBus = new CANBus("rio");
  public static final CANBus kCANivoreCANBus = new CANBus("main");

  // Talon FX IDs
  public static final int kExtension = 13;
  public static final int kIntake = 14;

  public static final int kIndexer = 15;
  public static final int kTriggers = 16;

  public static final int kTestLeftTriggers = 25; // TODO
  public static final int kTestRightTriggers = 26;

  public static final int kTestLeftLimitSwitch = 25; // TODO
  public static final int kTestRightLimitSwitch = 26;

  public static final int kTurret = 17;
  public static final int kHood = 18;
  public static final int kFlywheel = 19;
  public static final int kSecondFlywheel = 21;

  public static final int kHanger = 20;
}
