package frc.robot.lib.swerve.config;

import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/** Posicoes das cameras e limites conservadores para aceitar medidas AprilTag. */
public final class ConfigVision {
  private ConfigVision() {}

  public static final AprilTagFieldLayout FIELD_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  /*
   * IMPORTANTE: estas poses sao valores iniciais para simulacao.
   * Meça cada camera a partir do centro do robo e ajuste antes do uso real.
   * WPILib: +X frente, +Y esquerda, +Z cima; yaw positivo anti-horario.
   */
  public static final CameraConfig FRONT_CAMERA =
      camera("limelight-front", 0.30, 0.00, 0.55, 0.0, -15.0, 0.0);
  public static final CameraConfig LEFT_CAMERA =
      camera("limelight-left", 0.05, 0.27, 0.52, 0.0, -12.0, 35.0);
  public static final CameraConfig RIGHT_CAMERA =
      camera("limelight-right", 0.05, -0.27, 0.52, 0.0, -12.0, -35.0);

  public static final List<CameraConfig> CAMERAS =
      List.of(FRONT_CAMERA, LEFT_CAMERA, RIGHT_CAMERA);

  /** Processamento no robo; as Limelights fazem o trabalho pesado de imagem. */
  public static final double UPDATE_PERIOD_SECONDS = 1.0 / 30.0;
  public static final double TELEMETRY_PERIOD_SECONDS = 0.10;

  /** Limites de qualidade e plausibilidade. */
  public static final double MIN_TAG_DISTANCE_METERS = 0.15;
  public static final double MAX_SINGLE_TAG_DISTANCE_METERS = 4.5;
  public static final double MAX_MULTI_TAG_DISTANCE_METERS = 7.0;
  public static final double MAX_SINGLE_TAG_AMBIGUITY = 0.25;
  public static final double MAX_MEASUREMENT_AGE_SECONDS = 0.50;
  public static final double MAX_FUTURE_TIMESTAMP_SECONDS = 0.05;
  public static final double MAX_ROBOT_POSE_Z_METERS = 0.20;
  public static final double MAX_ROBOT_ROLL_PITCH_DEGREES = 12.0;
  public static final double MAX_GYRO_RATE_DEGREES_PER_SECOND = 360.0;
  public static final double FIELD_BORDER_MARGIN_METERS = 0.50;
  public static final double BASE_MAX_INNOVATION_METERS = 0.75;
  public static final double INNOVATION_PER_DISTANCE = 0.35;

  /** Cross-camera agreement gates. Cameras observing the same event are correlated. */
  public static final double CONSENSUS_MAX_TRANSLATION_METERS = 0.45;
  public static final double CONSENSUS_MAX_TIMESTAMP_DIFFERENCE_SECONDS = 0.060;
  public static final double SINGLE_CAMERA_STD_DEV_MULTIPLIER = 1.35;
  public static final double MULTI_CAMERA_STD_DEV_MULTIPLIER = 0.80;
  public static final double CAMERA_HEARTBEAT_TIMEOUT_SECONDS = 0.50;

  /** Modelo de incerteza: cresce aproximadamente com o quadrado da distancia. */
  public static final double MIN_XY_STD_DEV_METERS = 0.12;
  public static final double MAX_XY_STD_DEV_METERS = 2.50;
  public static final double BASE_XY_STD_DEV_METERS = 0.10;
  public static final double DISTANCE_STD_DEV_FACTOR = 0.055;
  public static final double MEGATAG2_THETA_STD_DEV_RADIANS = 9_999_999.0;

  /** Modelo visual simples usado apenas no simulador. */
  public static final double SIM_HORIZONTAL_FOV_DEGREES = 63.3;
  public static final double SIM_VERTICAL_FOV_DEGREES = 49.7;
  public static final double SIM_MAX_TAG_DISTANCE_METERS = 7.0;
  public static final double SIM_MAX_TAG_OBLIQUITY_DEGREES = 75.0;
  public static final int SIM_MAX_TAGS_PER_CAMERA = 8;
  public static final double SIM_BASE_TRANSLATION_NOISE_METERS = 0.015;
  public static final double SIM_DISTANCE_NOISE_FACTOR = 0.004;
  public static final double SIM_YAW_NOISE_DEGREES = 0.25;
  public static final double SIM_LATENCY_MILLISECONDS = 22.0;

  private static CameraConfig camera(
      String name,
      double xMeters,
      double yMeters,
      double zMeters,
      double rollDegrees,
      double pitchDegrees,
      double yawDegrees) {
    return new CameraConfig(
        name,
        new Transform3d(
            new Translation3d(xMeters, yMeters, zMeters),
            new Rotation3d(
                Units.degreesToRadians(rollDegrees),
                Units.degreesToRadians(pitchDegrees),
                Units.degreesToRadians(yawDegrees))),
        SIM_HORIZONTAL_FOV_DEGREES,
        SIM_VERTICAL_FOV_DEGREES,
        SIM_MAX_TAG_DISTANCE_METERS);
  }

  public record CameraConfig(
      String name,
      Transform3d robotToCamera,
      double horizontalFovDegrees,
      double verticalFovDegrees,
      double maxTagDistanceMeters) {
    public CameraConfig {
      if (name == null || name.isBlank() || robotToCamera == null) {
        throw new IllegalArgumentException("Camera precisa de nome e transformacao.");
      }
      if (horizontalFovDegrees <= 0.0
          || verticalFovDegrees <= 0.0
          || maxTagDistanceMeters <= 0.0) {
        throw new IllegalArgumentException("FOV e alcance da camera precisam ser positivos.");
      }
    }
  }
}
