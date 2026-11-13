package frc.robot.config;

import java.util.List;
import java.util.stream.IntStream;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class FieldTargets {
  public static final double LENGTH = ConfigVision.FIELD_LAYOUT.getFieldLength();
  public static final double WIDTH = ConfigVision.FIELD_LAYOUT.getFieldWidth();
  public static final double INTAKE_EXTENSION = 0.16;
  public static final double BUMPER_LENGTH = 0.762;
  public static final double BUMPER_WIDTH = 0.762;
  public static final double SWEPT_RADIUS =
      Math.hypot(BUMPER_LENGTH / 2 + INTAKE_EXTENSION, BUMPER_WIDTH / 2);

  // Origem WPILib azul. Vermelho recebe a rotação de 180° do campo.
  private static final double[] SHOT_Y = {1.5, 2.65, 5.4, 6.55};
  private static final double SHOT_X = 2.75;

  private FieldTargets() {}

  public static Translation2d hub(Alliance alliance) {
    return alliance == Alliance.Red ? ConfigAutomaticAim.RED_HUB : ConfigAutomaticAim.BLUE_HUB;
  }

  public static List<Pose2d> shootingPoints(Alliance alliance) {
    return IntStream.range(0, SHOT_Y.length).mapToObj(index -> {
      double y = SHOT_Y[index];
      Translation2d point = alliance == Alliance.Red
          ? new Translation2d(LENGTH - SHOT_X, WIDTH - y) : new Translation2d(SHOT_X, y);
      return new Pose2d(point, hub(alliance).minus(point).getAngle()
          .minus(Rotation2d.fromRadians(ConfigAutomaticAim.SHOOTER_YAW_OFFSET_RADIANS)));
    }).toList();
  }

  public static boolean insideScoringZone(Pose2d pose, Alliance alliance) {
    double x = alliance == Alliance.Red ? LENGTH - pose.getX() : pose.getX();
    return x >= SWEPT_RADIUS && x <= ConfigAutomaticAim.ALLIANCE_ZONE_DEPTH_METERS - SWEPT_RADIUS
        && pose.getY() >= SWEPT_RADIUS && pose.getY() <= WIDTH - SWEPT_RADIUS;
  }
}

