package frc.robot.lib.swerve.localization;

import edu.wpi.first.math.MathUtil;
import frc.robot.lib.swerve.config.ConfigLocalization;

/** Pure scoring function so collision heuristics can be replay-tested. */
public final class CollisionConfidence {
  private CollisionConfidence() {}

  public static double calculate(
      double requestedSpeed,
      double measuredSpeed,
      double imuAcceleration,
      double jerk,
      double wheelAcceleration) {
    double impactScore =
        Math.max(
            imuAcceleration / ConfigLocalization.IMPACT_ACCELERATION_METERS_PER_SECOND_SQUARED,
            jerk / ConfigLocalization.IMPACT_JERK_METERS_PER_SECOND_CUBED);
    double stallScore =
        requestedSpeed < ConfigLocalization.MIN_COMMANDED_SPEED_FOR_STALL_METERS_PER_SECOND
            ? 0.0
            : 1.0 - measuredSpeed / Math.max(0.1, requestedSpeed);
    double slipScore =
        wheelAcceleration < ConfigLocalization.MIN_WHEEL_ACCELERATION_FOR_SLIP
            ? 0.0
            : 1.0 - imuAcceleration / Math.max(0.1, wheelAcceleration);
    return MathUtil.clamp(0.45 * impactScore + 0.35 * stallScore + 0.20 * slipScore, 0.0, 1.0);
  }
}
