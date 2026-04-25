// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.Geoffrey;

/** Add your docs here. */
public class TrajectoryConfig {

  public double G = 9.80;
  public double DY = 1.2288; // 1.8288 - 0.6
  public boolean APEX_BEFORE_TARGET = true;

  public double min_hMax = 0;
  public double max_hMax = 4;

  // Degree-3 polynomial coefficients for aero correction
  // Features: [1, dx, hmax, dx^2, dx*hmax, hmax^2, dx^3, dx^2*hmax, dx*hmax^2, hmax^3]
  public double[] DVX_COEFFS = {
    -0.08450618093881918,
    -0.009837347218053185,
    0.13674748925075123,
    -0.0030658470551465714,
    0.018822387782098767,
    -0.018255996814091676,
    -0.00011147978768855354,
    0.0010860553848757643,
    -0.003321669210630178,
    0.0036514134394532915
  };
  public double[] DVY_COEFFS = {
    -0.07230402829128711,
    -0.028479659883815272,
    0.088710520507098,
    -0.001715989222679634,
    0.003342169837324043,
    -0.018125628513105257,
    -0.00013078819069747838,
    0.0007362494446703192,
    -0.0015496733072326574,
    0.0024913578085115125
  };

  public static TrajectoryConfig getHubConfig() {
    TrajectoryConfig config = new TrajectoryConfig();
    config.G = 9.80;
    config.DY = 1.2288; // 1.8288 - 0.6
    config.APEX_BEFORE_TARGET = true;
    config.min_hMax = config.DY + 0.1;
    config.max_hMax = config.DY + 3.0;

    config.DVX_COEFFS =
        new double[] {
          0.013398884570489489,
          0.022107915517827894,
          0.0029427282642041606,
          0.0017206547342406172,
          0.0019263019981883502,
          0.009334522168364865,
          -1.2511226840875717e-05,
          0.0001401685261463257,
          -0.0005130834479621883,
          -0.00014818788954227852
        };
    config.DVY_COEFFS =
        new double[] {
          -0.04869114419294457,
          -0.01693698349406852,
          0.08914093527901754,
          -0.000800205995647986,
          0.008695002645675872,
          -0.019414830459831084,
          -0.0001293905540209403,
          0.0007289722453766188,
          -0.0023150786885729564,
          0.003554841170547695
        };
    return config;
  }

  public static TrajectoryConfig getPassingConfig() {
    TrajectoryConfig config = new TrajectoryConfig();
    config.G = 9.80;
    config.DY = -0.6;
    config.APEX_BEFORE_TARGET = true;
    config.min_hMax = 0.4;
    config.max_hMax = 4;

    config.DVX_COEFFS =
        new double[] {
          -0.1779035659344022,
          -0.04328385029259351,
          0.7363934104772497,
          -0.013401210967835423,
          0.14866841663923833,
          -0.6183758086494308,
          -0.00023715163020891078,
          0.006482624550329427,
          -0.04818115086231098,
          0.13956477423568728
        };
    config.DVY_COEFFS =
        new double[] {
          0.016181735656721236,
          -0.011739688041672882,
          0.0030192687547994416,
          -0.0024250197964459817,
          0.0021606067912207927,
          0.005920547896335245,
          -1.4916565228371287e-06,
          0.0003994230814203738,
          -0.0014056025691093217,
          0.00028463151742891827
        };
    return config;
  }
}
