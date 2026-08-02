package frc.robot.config;

/** Ajustes do monitor de odometria, colisao, desempenho e PathPlanner. */
public final class ConfigLocalization {
  private ConfigLocalization() {}

  /** Historico fixo: uma amostra a cada 20 ms por dois segundos. */
  public static final double SAMPLE_PERIOD_SECONDS = 0.020;
  public static final double HISTORY_SECONDS = 2.0;
  public static final int HISTORY_CAPACITY =
      (int) Math.ceil(HISTORY_SECONDS / SAMPLE_PERIOD_SECONDS) + 1;

  /** Heuristicas conservadoras; precisam ser ajustadas com logs do robo real. */
  public static final double IMPACT_ACCELERATION_METERS_PER_SECOND_SQUARED = 3.5;
  public static final double IMPACT_JERK_METERS_PER_SECOND_CUBED = 30.0;
  public static final double MIN_COMMANDED_SPEED_FOR_STALL_METERS_PER_SECOND = 1.0;
  public static final double MAX_MEASURED_TO_COMMANDED_STALL_RATIO = 0.25;
  public static final double STALL_DEBOUNCE_SECONDS = 0.30;
  public static final double MIN_WHEEL_ACCELERATION_FOR_SLIP = 2.0;
  public static final double MAX_IMU_TO_WHEEL_ACCELERATION_RATIO = 0.20;
  public static final double SLIP_DEBOUNCE_SECONDS = 0.12;
  public static final double RECOVERY_STABLE_SECONDS = 0.40;
  public static final double STABLE_ACCELERATION_METERS_PER_SECOND_SQUARED = 0.60;
  public static final int TRUSTED_POSE_LOOKBACK_SAMPLES = 5;

  /*
   * Um rollback automatico pode piorar a pose se outro robo realmente deslocar o chassi.
   * A infraestrutura e o comando manual existem, mas a automacao fica desligada por seguranca.
   */
  public static final boolean ENABLE_AUTOMATIC_COLLISION_ROLLBACK = false;
  public static final double MAX_MANUAL_ROLLBACK_DISTANCE_METERS = 0.50;

  /** Trava X quando pontuando e durante bloqueio confirmado. */
  public static final boolean LOCK_X_WHILE_SCORING = true;

  /** Ganhos iniciais do controlador holonomico PathPlanner. */
  public static final double PATH_TRANSLATION_KP = 5.0;
  public static final double PATH_ROTATION_KP = 4.0;

  /** Telemetria e alertas de desempenho. */
  public static final double PERFORMANCE_TELEMETRY_PERIOD_SECONDS = 0.20;
  public static final double LOOP_WARNING_SECONDS = 0.030;
  public static final double CAN_WARNING_UTILIZATION = 0.85;
  public static final double CPU_TEMPERATURE_WARNING_CELSIUS = 75.0;
}
