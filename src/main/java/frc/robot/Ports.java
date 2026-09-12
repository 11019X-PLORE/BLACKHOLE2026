package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Ports {
  // CAN Buses
  public static final CANBus kRoboRioCANBus = new CANBus("rio");
  public static final CANBus kCANivoreCANBus = new CANBus("main");

  // Talon FX IDs (offseason robot)
  public static final int kIndexer = 17; // roller indexer (1 motor)
  public static final int kTrigger = 19; // trigger motor (1 motor)

  public static final int kArm = 13; // arm: deploys intake + controls shooting angle
  public static final int kFlywheel0 = 14;
  public static final int kFlywheel1 = 15;
  public static final int kFlywheel2 = 16;

  public static final int kBlocker = 18; // blocker: angle control (1 motor)
}
