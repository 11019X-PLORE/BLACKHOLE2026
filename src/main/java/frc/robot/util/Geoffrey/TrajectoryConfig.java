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
    config.min_hMax = 1;
    config.max_hMax = 6;

    config.DVX_COEFFS =
        new double[] {
          -0.048256378229883934,
          0.004645128298960892,
          0.13747762243110465,
          -0.00040371047867558116,
          0.02484219323204584,
          -0.06742718947666232,
          -8.878625075056483e-05,
          0.0010411682865853209,
          -0.0053796383747517435,
          0.010493765902012911
        };
    config.DVY_COEFFS =
        new double[] {
          0.002882918553756983,
          -0.005400324795804963,
          0.028904765361434576,
          -0.0006073784724201789,
          0.0025587899416334324,
          0.002487259134709766,
          -3.407548742215272e-05,
          0.00031739275665163447,
          -0.0009985960449592633,
          0.0008698790342732325
        };
    return config;
  }
}
