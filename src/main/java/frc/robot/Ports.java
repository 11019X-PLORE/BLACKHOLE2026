package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Ports {
  // CAN Buses
  public static final CANBus kRoboRioCANBus = new CANBus("rio");
  public static final CANBus kCANivoreCANBus = new CANBus("main");

  // Talon FX IDs
  public static final int kIntakeArm = 13;
  public static final int kIntakeRollers = 14;

  public static final int kRotatorRollers = 15;
  public static final int kIndexerRollers = 16;

  public static final int kTurret = 17;
  public static final int kHood = 18;
  public static final int kFlywheel = 19;
  public static final int kSecondFlywheel = 21;

  public static final int kHanger = 20;
}
