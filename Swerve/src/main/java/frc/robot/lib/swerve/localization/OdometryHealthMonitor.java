package frc.robot.lib.swerve.localization;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import java.util.Optional;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.swerve.config.ConfigLocalization;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/**
 * Mantem historico de 20 ms e identifica impactos/bloqueios sem assumir que o IMU mede velocidade.
 *
 * <p>As heuristicas sao deliberadamente conservadoras. Rodas podem girar contra uma parede e o
 * Pigeon mede aceleracao, nao velocidade translacional; por isso a pose de recuperacao e apenas uma
 * recomendacao e o rollback automatico vem desligado.
 */
public final class OdometryHealthMonitor extends SubsystemBase {
  public enum HealthState {
    NOMINAL,
    IMPACT_SUSPECTED,
    BLOCKED,
    RECOVERING
  }

  private final SwerveSubsystem drivetrain;
  private final PoseSample[] history = new PoseSample[ConfigLocalization.HISTORY_CAPACITY];
  private final StatusSignal<LinearAcceleration> accelerationX;
  private final StatusSignal<LinearAcceleration> accelerationY;
  private int nextHistoryIndex;
  private int historySize;
  private boolean recoveryPoseAvailable;
  private double recoveryXMeters;
  private double recoveryYMeters;
  private double recoveryHeadingRadians;
  private HealthState healthState = HealthState.NOMINAL;
  private double requestedVelocityX;
  private double requestedVelocityY;
  private double requestedOmega;
  private double previousMeasuredSpeed;
  private double previousImuAcceleration;
  private double lastSampleSeconds = Double.NEGATIVE_INFINITY;
  private double stateStartSeconds;
  private double stallStartSeconds = Double.NaN;
  private double slipStartSeconds = Double.NaN;
  private double stableStartSeconds = Double.NaN;
  private double lastTelemetrySeconds = Double.NEGATIVE_INFINITY;
  private double lastImuAcceleration;
  private double lastJerk;
  private double lastMeasuredSpeed;
  private double collisionConfidence;
  private double accelerationBiasX;
  private double accelerationBiasY;

  public OdometryHealthMonitor(SwerveSubsystem drivetrain) {
    this.drivetrain = drivetrain;
    for (int i = 0; i < history.length; i++) {
      history[i] = new PoseSample();
    }
    accelerationX = drivetrain.getPigeon2().getAccelerationX();
    accelerationY = drivetrain.getPigeon2().getAccelerationY();
    accelerationX.setUpdateFrequency(50.0);
    accelerationY.setUpdateFrequency(50.0);
  }

  /** Registra a intencao do controlador; chame tambem nos comandos autonomos. */
  public void setRequestedSpeeds(double velocityX, double velocityY, double omegaRadiansPerSecond) {
    requestedVelocityX = velocityX;
    requestedVelocityY = velocityY;
    requestedOmega = omegaRadiansPerSecond;
  }

  public void setRequestedSpeeds(ChassisSpeeds speeds) {
    setRequestedSpeeds(
        speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
  }

  public HealthState getHealthState() {
    return healthState;
  }

  /** Solicita X-lock somente depois de um bloqueio confirmado. */
  public boolean shouldHoldPosition() {
    return healthState == HealthState.BLOCKED;
  }

  public Optional<Pose2d> getRecommendedRecoveryPose() {
    if (!recoveryPoseAvailable) {
      return Optional.empty();
    }
    Pose2d current = drivetrain.getState().Pose;
    Pose2d candidate =
        new Pose2d(
            recoveryXMeters,
            recoveryYMeters,
            Rotation2d.fromRadians(recoveryHeadingRadians));
    if (current.getTranslation().getDistance(candidate.getTranslation())
        > ConfigLocalization.MAX_MANUAL_ROLLBACK_DISTANCE_METERS) {
      return Optional.empty();
    }
    return Optional.of(candidate);
  }

  /** Retorna true somente quando havia uma pose historica plausivel para aplicar. */
  public boolean applyRecommendedRecoveryPose() {
    Optional<Pose2d> recoveryPose = getRecommendedRecoveryPose();
    if (recoveryPose.isEmpty()) {
      return false;
    }
    drivetrain.resetPose(recoveryPose.get());
    transitionTo(HealthState.RECOVERING, Timer.getFPGATimestamp());
    return true;
  }

  @Override
  public void periodic() {
    double nowSeconds = Timer.getFPGATimestamp();
    if (nowSeconds - lastSampleSeconds < ConfigLocalization.SAMPLE_PERIOD_SECONDS) {
      return;
    }
    double dt =
        Double.isFinite(lastSampleSeconds)
            ? Math.max(0.001, nowSeconds - lastSampleSeconds)
            : ConfigLocalization.SAMPLE_PERIOD_SECONDS;
    lastSampleSeconds = nowSeconds;
    if (DriverStation.isDisabled()) {
      requestedVelocityX = 0.0;
      requestedVelocityY = 0.0;
      requestedOmega = 0.0;
    }

    BaseStatusSignal.refreshAll(accelerationX, accelerationY);
    double ax = accelerationX.getValue().in(MetersPerSecondPerSecond);
    double ay = accelerationY.getValue().in(MetersPerSecondPerSecond);
    if (Math.hypot(requestedVelocityX, requestedVelocityY) < 0.10
        && Math.hypot(
                drivetrain.getState().Speeds.vxMetersPerSecond,
                drivetrain.getState().Speeds.vyMetersPerSecond)
            < 0.10) {
      accelerationBiasX += 0.01 * (ax - accelerationBiasX);
      accelerationBiasY += 0.01 * (ay - accelerationBiasY);
    }
    ax -= accelerationBiasX;
    ay -= accelerationBiasY;
    double imuAcceleration = Math.hypot(ax, ay);
    double jerk = Math.abs(imuAcceleration - previousImuAcceleration) / dt;

    ChassisSpeeds measured = drivetrain.getState().Speeds;
    double measuredSpeed = Math.hypot(measured.vxMetersPerSecond, measured.vyMetersPerSecond);
    double requestedSpeed = Math.hypot(requestedVelocityX, requestedVelocityY);
    double wheelAcceleration = Math.abs(measuredSpeed - previousMeasuredSpeed) / dt;
    collisionConfidence =
        CollisionConfidence.calculate(
            requestedSpeed, measuredSpeed, imuAcceleration, jerk, wheelAcceleration);
    previousMeasuredSpeed = measuredSpeed;
    previousImuAcceleration = imuAcceleration;
    lastImuAcceleration = imuAcceleration;
    lastJerk = jerk;
    lastMeasuredSpeed = measuredSpeed;

    saveSample(nowSeconds, drivetrain.getState().Pose, measured);

    boolean impact =
        imuAcceleration >= ConfigLocalization.IMPACT_ACCELERATION_METERS_PER_SECOND_SQUARED
            || jerk >= ConfigLocalization.IMPACT_JERK_METERS_PER_SECOND_CUBED;
    boolean stallCandidate =
        requestedSpeed
                >= ConfigLocalization.MIN_COMMANDED_SPEED_FOR_STALL_METERS_PER_SECOND
            && measuredSpeed
                <= requestedSpeed
                    * ConfigLocalization.MAX_MEASURED_TO_COMMANDED_STALL_RATIO;
    boolean slipCandidate =
        wheelAcceleration >= ConfigLocalization.MIN_WHEEL_ACCELERATION_FOR_SLIP
            && imuAcceleration
                <= wheelAcceleration
                    * ConfigLocalization.MAX_IMU_TO_WHEEL_ACCELERATION_RATIO;
    boolean stalled = debounced(stallCandidate, nowSeconds, true);
    boolean slipping = debounced(slipCandidate, nowSeconds, false);
    boolean stable =
        imuAcceleration <= ConfigLocalization.STABLE_ACCELERATION_METERS_PER_SECOND_SQUARED;

    switch (healthState) {
      case NOMINAL -> {
        if (impact) {
          captureRecoveryPose();
          transitionTo(HealthState.IMPACT_SUSPECTED, nowSeconds);
        } else if (stalled || collisionConfidence >= 0.70) {
          captureRecoveryPose();
          transitionTo(HealthState.IMPACT_SUSPECTED, nowSeconds);
        }
      }
      case IMPACT_SUSPECTED -> {
        if (stalled || slipping || (impact && nowSeconds - stateStartSeconds > 0.04)) {
          transitionTo(HealthState.BLOCKED, nowSeconds);
          if (ConfigLocalization.ENABLE_AUTOMATIC_COLLISION_ROLLBACK) {
            applyRecommendedRecoveryPose();
          }
        } else if (stableFor(stable, nowSeconds, 0.25)) {
          transitionTo(HealthState.NOMINAL, nowSeconds);
        }
      }
      case BLOCKED -> {
        // O piloto precisa soltar a translacao antes de o solver liberar o X-lock.
        if (requestedSpeed < 0.20 && stableFor(stable, nowSeconds, 0.20)) {
          transitionTo(HealthState.RECOVERING, nowSeconds);
        }
      }
      case RECOVERING -> {
        if (stableFor(stable, nowSeconds, ConfigLocalization.RECOVERY_STABLE_SECONDS)) {
          transitionTo(HealthState.NOMINAL, nowSeconds);
        } else if (impact) {
          transitionTo(HealthState.IMPACT_SUSPECTED, nowSeconds);
        }
      }
    }

    if (nowSeconds - lastTelemetrySeconds >= 0.10) {
      lastTelemetrySeconds = nowSeconds;
      publishTelemetry(requestedSpeed);
    }
  }

  private boolean debounced(boolean condition, double nowSeconds, boolean stall) {
    double start = stall ? stallStartSeconds : slipStartSeconds;
    if (!condition) {
      if (stall) {
        stallStartSeconds = Double.NaN;
      } else {
        slipStartSeconds = Double.NaN;
      }
      return false;
    }
    if (!Double.isFinite(start)) {
      start = nowSeconds;
      if (stall) {
        stallStartSeconds = start;
      } else {
        slipStartSeconds = start;
      }
    }
    double required =
        stall
            ? ConfigLocalization.STALL_DEBOUNCE_SECONDS
            : ConfigLocalization.SLIP_DEBOUNCE_SECONDS;
    return nowSeconds - start >= required;
  }

  private boolean stableFor(boolean stable, double nowSeconds, double requiredSeconds) {
    if (!stable) {
      stableStartSeconds = Double.NaN;
      return false;
    }
    if (!Double.isFinite(stableStartSeconds)) {
      stableStartSeconds = nowSeconds;
    }
    return nowSeconds - stableStartSeconds >= requiredSeconds;
  }

  private void transitionTo(HealthState nextState, double nowSeconds) {
    if (healthState == nextState) {
      return;
    }
    healthState = nextState;
    stateStartSeconds = nowSeconds;
    stableStartSeconds = Double.NaN;
    if (nextState == HealthState.NOMINAL) {
      recoveryPoseAvailable = false;
    }
  }

  private void saveSample(double timestamp, Pose2d pose, ChassisSpeeds speeds) {
    PoseSample sample = history[nextHistoryIndex];
    sample.timestampSeconds = timestamp;
    sample.xMeters = pose.getX();
    sample.yMeters = pose.getY();
    sample.headingRadians = pose.getRotation().getRadians();
    sample.velocityX = speeds.vxMetersPerSecond;
    sample.velocityY = speeds.vyMetersPerSecond;
    sample.omega = speeds.omegaRadiansPerSecond;
    nextHistoryIndex = (nextHistoryIndex + 1) % history.length;
    historySize = Math.min(history.length, historySize + 1);
  }

  /**
   * Reconstroi a pose anterior ao impacto a partir de uma amostra confiavel e das velocidades
   * relativas salvas. A ultima amostra, potencialmente contaminada pelo impacto, nao e integrada.
   */
  private void captureRecoveryPose() {
    int lookback = ConfigLocalization.TRUSTED_POSE_LOOKBACK_SAMPLES;
    if (historySize <= lookback) {
      recoveryPoseAvailable = false;
      return;
    }

    int startIndex = floorMod(nextHistoryIndex - 1 - lookback, history.length);
    PoseSample previous = history[startIndex];
    double x = previous.xMeters;
    double y = previous.yMeters;
    double heading = previous.headingRadians;
    for (int step = 1; step < lookback; step++) {
      int index = (startIndex + step) % history.length;
      PoseSample current = history[index];
      double dt =
          Math.max(
              0.0,
              Math.min(
                  0.040, current.timestampSeconds - previous.timestampSeconds));
      double cosine = Math.cos(heading);
      double sine = Math.sin(heading);
      x += (previous.velocityX * cosine - previous.velocityY * sine) * dt;
      y += (previous.velocityX * sine + previous.velocityY * cosine) * dt;
      heading += previous.omega * dt;
      previous = current;
    }
    recoveryXMeters = x;
    recoveryYMeters = y;
    recoveryHeadingRadians = heading;
    recoveryPoseAvailable = true;
  }

  private void publishTelemetry(double requestedSpeed) {
    SmartDashboard.putString("OdometryHealth/State", healthState.name());
    SmartDashboard.putNumber("OdometryHealth/HistorySamples", historySize);
    SmartDashboard.putNumber("OdometryHealth/RequestedSpeedMps", requestedSpeed);
    SmartDashboard.putNumber("OdometryHealth/MeasuredWheelSpeedMps", lastMeasuredSpeed);
    SmartDashboard.putNumber("OdometryHealth/CollisionConfidence", collisionConfidence);
    SmartDashboard.putNumber("OdometryHealth/ImuAccelerationMps2", lastImuAcceleration);
    SmartDashboard.putNumber("OdometryHealth/JerkMps3", lastJerk);
    SmartDashboard.putBoolean(
        "OdometryHealth/RecoveryPoseAvailable", getRecommendedRecoveryPose().isPresent());
  }

  private static int floorMod(int value, int modulus) {
    int result = value % modulus;
    return result < 0 ? result + modulus : result;
  }

  private static final class PoseSample {
    double timestampSeconds;
    double xMeters;
    double yMeters;
    double headingRadians;
    double velocityX;
    double velocityY;
    double omega;
  }
}
