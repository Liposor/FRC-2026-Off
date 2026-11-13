package frc.robot.config;

import edu.wpi.first.math.MathUtil;

/** Dados do STORM de Houston. Ganhos de partida para validação no robô. */
public final class ConfigMechanisms {
  private ConfigMechanisms() {}

  public static final int INTAKE_ID = 32;
  public static final int LOWER_ROLLER_ID = 21;
  public static final int LEFT_ROLLER_ID = 22;
  public static final int RIGHT_ROLLER_ID = 23;
  public static final int RIGHT_SHOOTER_ID = 24;
  public static final int LEFT_SHOOTER_ID = 25;
  public static final int FLAP_ID = 61;

  public static final double COLLECT_RPS = 85;
  public static final double MAX_COLLECT_RPS = 90;
  public static final double EJECT_INTAKE_RPS = -35;
  public static final double FEED_UPPER_RPS = 30;
  public static final double EJECT_UPPER_RPS = -15;
  public static final double FEED_LOWER_VOLTS = -7.2;
  public static final double EJECT_LOWER_VOLTS = 5;
  public static final double FLYWHEEL_KV = 0.13;
  public static final double ROLLER_KV = 0.13;
  public static final double REDUCED_SHOT_RPM = 150;
  public static final double FLAP_MAX_DEGREES = 40;
  public static final double FLAP_REDUCTION = 10;
  public static final double FLAP_MAX_OUTPUT = 0.18;
  public static final int FLAP_CURRENT_LIMIT_AMPS = 15;
  public static final double RPM_READY_TOLERANCE = 130;
  public static final double FLAP_READY_TOLERANCE_DEGREES = 1.5;

  private static final double[][] HUB_MAP = {
    {1, 1450, 10}, {1.5, 1450, 10}, {2, 1550, 10}, {2.5, 1700, 10},
    {3, 1800, 13}, {3.5, 1950, 13}, {4, 1950, 17}, {4.5, 2100, 18},
    {5, 2350, 21}, {8, 2850, 27}, {15, 5700, 40}
  };

  public static Shot shotAt(double distanceMeters, double rpmOffset, double flapOffsetDegrees) {
    int upper = 1;
    while (upper < HUB_MAP.length - 1 && distanceMeters > HUB_MAP[upper][0]) {
      upper++;
    }
    double[] low = HUB_MAP[upper - 1];
    double[] high = HUB_MAP[upper];
    double fraction = MathUtil.clamp((distanceMeters - low[0]) / (high[0] - low[0]), 0, 1);
    return new Shot(
        MathUtil.clamp(MathUtil.interpolate(low[1], high[1], fraction) + rpmOffset, 0, 5500),
        MathUtil.clamp(MathUtil.interpolate(low[2], high[2], fraction) + flapOffsetDegrees,
            0, FLAP_MAX_DEGREES));
  }

  public record Shot(double rpm, double flapDegrees) {}
}

