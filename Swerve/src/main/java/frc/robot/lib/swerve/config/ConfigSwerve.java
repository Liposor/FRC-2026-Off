package frc.robot.lib.swerve.config;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

/**
 * Ajustes de comportamento do swerve.
 *
 * <p>As constantes eletricas, IDs CAN, reducoes e offsets absolutos continuam em
 * {@code TunerConstants}, pois devem vir do Phoenix Tuner X. Este arquivo concentra os valores
 * que normalmente sao alterados durante o desenvolvimento do comportamento do robo.
 */
public final class ConfigSwerve {
  private ConfigSwerve() {}

  /** Velocidade fisica medida com 12 V, usada pelo modelo e pelo controle Phoenix. */
  public static final LinearVelocity SPEED_AT_12_VOLTS =
      SwerveHardwareConfig.DRIVE_RATIO.theoreticalFreeSpeed();

  /** Limites máximos aplicados ao controle do piloto. */
  public static final LinearVelocity TELEOP_MAX_SPEED = MetersPerSecond.of(4.5);
  public static final AngularVelocity TELEOP_MAX_ANGULAR_RATE =
      RotationsPerSecond.of(0.75);

  /** Deadband proporcional dos eixos do controle, entre 0.0 e 1.0. */
  public static final double JOYSTICK_DEADBAND = 0.10;

  /** Periodo da simulacao Phoenix; 4 ms deixa os controladores mais estaveis. */
  public static final double SIM_LOOP_PERIOD_SECONDS = 0.004;

  /** Acceleration and steering limits used to keep commands physically achievable. */
  public static final double MAX_LINEAR_ACCELERATION_METERS_PER_SECOND_SQUARED = 5.0;
  public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 12.0;
  public static final double MAX_STEER_RATE_ROTATIONS_PER_SECOND = 2.0;

  /** Valores usados pelos autonomos simples de exemplo. */
  public static final double AUTO_TRANSLATION_SPEED_METERS_PER_SECOND = 1.0;
  public static final double AUTO_STRAIGHT_DURATION_SECONDS = 2.0;
  public static final double AUTO_SQUARE_SIDE_DURATION_SECONDS = 1.0;
  public static final double AUTO_BRAKE_DURATION_SECONDS = 0.25;

  /*
   * Perfis escolhidos automaticamente pela Superstructure.
   * Cada escala multiplica os limites TELEOP_MAX_* acima.
   */
  public static final DriveProfile NORMAL_PROFILE = new DriveProfile("Normal", 1.00, 1.00);
  public static final DriveProfile COLLECTING_PROFILE =
      new DriveProfile("Coletando", 0.45, 0.35);
  public static final DriveProfile HOLDING_PROFILE =
      new DriveProfile("Com peca", 0.75, 0.65);
  public static final DriveProfile SCORING_PROFILE =
      new DriveProfile("Pontuando", 0.35, 0.30);
  public static final DriveProfile CLIMBING_PROFILE =
      new DriveProfile("Escalando", 0.20, 0.20);

  /** Limites relativos de conducao associados a um estado do robo. */
  public record DriveProfile(String name, double translationScale, double rotationScale) {
    public DriveProfile {
      if (name == null || name.isBlank()) {
        throw new IllegalArgumentException("O perfil precisa de um nome.");
      }
      validateScale("translationScale", translationScale);
      validateScale("rotationScale", rotationScale);
    }

    private static void validateScale(String field, double value) {
      if (!Double.isFinite(value) || value < 0.0 || value > 1.0) {
        throw new IllegalArgumentException(field + " deve estar entre 0.0 e 1.0.");
      }
    }
  }
}
