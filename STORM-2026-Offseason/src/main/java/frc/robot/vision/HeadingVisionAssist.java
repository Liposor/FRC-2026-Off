package frc.robot.vision;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import frc.robot.config.ConfigVision;

/** Estima o erro entre o heading de campo e o MT1 com ambos no instante da captura. */
final class HeadingVisionAssist {
  enum Status { OFF, WAITING, TRACKING, HOLDING }

  private record GyroSample(Rotation2d yaw, double maxRate) {}

  private final TimeInterpolatableBuffer<GyroSample> history = TimeInterpolatableBuffer.createBuffer(
      (first, second, fraction) -> new GyroSample(first.yaw().interpolate(second.yaw(), fraction),
          Math.max(first.maxRate(), second.maxRate())), 0.4);
  private final Map<String, Double> consumedFrames = new HashMap<>();
  private boolean enabled;
  private boolean teleop;
  private double acceptAfter;
  private double nextEvaluation;
  private double lastAccepted = Double.NEGATIVE_INFINITY;
  private double offsetDegrees;
  private double errorSum;
  private double minError;
  private double maxError;
  private int newFrameCount;

  boolean enabled() { return enabled; }
  double offsetDegrees() { return offsetDegrees; }

  void setEnabled(boolean requested, double now) {
    if (enabled != requested) {
      enabled = requested;
      clearFrames(now);
    }
  }

  void setTeleop(boolean active, double now) {
    if (active != teleop) {
      teleop = active;
      clearFrames(now);
    }
  }

  void resetReference(double now) {
    offsetDegrees = 0;
    clearFrames(now);
  }

  private void clearFrames(double now) {
    acceptAfter = now;
    nextEvaluation = now;
    lastAccepted = Double.NEGATIVE_INFINITY;
    history.clear();
    consumedFrames.clear();
  }

  Status status(double now) {
    if (!enabled) return Status.OFF;
    if (teleop && now - lastAccepted <= ConfigVision.HEADING_ASSIST_HOLD_AFTER_SECONDS) {
      return Status.TRACKING;
    }
    return offsetDegrees != 0 ? Status.HOLDING : Status.WAITING;
  }

  void recordGyro(double timestamp, Rotation2d yaw, double rate) {
    if (enabled && teleop && timestamp >= acceptAfter && Double.isFinite(yaw.getRadians())
        && Double.isFinite(rate)) {
      history.addSample(timestamp, new GyroSample(yaw, Math.abs(rate)));
    }
  }

  boolean beginEvaluation(double now, double currentYawRate) {
    if (!enabled || !teleop || now < nextEvaluation || !slowEnough(currentYawRate)) {
      return false;
    }
    nextEvaluation = now + ConfigVision.HEADING_ASSIST_PERIOD_SECONDS;
    errorSum = 0;
    minError = Double.POSITIVE_INFINITY;
    maxError = Double.NEGATIVE_INFINITY;
    newFrameCount = 0;
    return true;
  }

  void consider(VisionObservation frame, Rotation2d gyroToField, double now) {
    double capturedAt = frame.timestampSeconds();
    if (capturedAt <= acceptAfter || now - capturedAt > ConfigVision.HEADING_ASSIST_MAX_FRAME_AGE_SECONDS
        || !hasTwoDistinctTags(frame) || frame.averageTagDistanceMeters() > ConfigVision.HEADING_ASSIST_MAX_DISTANCE_METERS
        || frame.maximumAmbiguity() < 0 || frame.maximumAmbiguity() > ConfigVision.HEADING_ASSIST_MAX_AMBIGUITY
        || !VisionHeadingResetter.isGeometricallyPlausible(frame, now, ConfigVision.FIELD_LAYOUT)) {
      return;
    }
    var samples = history.getInternalBuffer();
    var before = samples.floorEntry(capturedAt);
    var after = samples.ceilingEntry(capturedAt);
    if (before == null || after == null
        || after.getKey() - before.getKey() > ConfigVision.HEADING_ASSIST_MAX_HISTORY_GAP_SECONDS
        || !slowEnough(Math.max(before.getValue().maxRate(), after.getValue().maxRate()))) {
      return;
    }
    Rotation2d expected = history.getSample(capturedAt).orElseThrow().yaw().plus(gyroToField);
    double error = frame.robotPose().toPose2d().getRotation().minus(expected).getDegrees();
    // Frames já vistos podem vetar discordância entre câmeras, mas nunca iniciar outro passo.
    if (capturedAt > consumedFrames.getOrDefault(frame.cameraName(), Double.NEGATIVE_INFINITY)) {
      consumedFrames.put(frame.cameraName(), capturedAt);
      errorSum += error;
      newFrameCount++;
    }
    // Limitar só depois de medir a discordância evita ignorar uma câmera divergente.
    minError = Math.min(minError, error);
    maxError = Math.max(maxError, error);
  }

  double finishEvaluation(double now) {
    if (newFrameCount == 0
        || maxError - minError > ConfigVision.HEADING_ASSIST_MAX_CAMERA_SPREAD_DEGREES
        || Math.max(Math.abs(minError), Math.abs(maxError)) > ConfigVision.HEADING_ASSIST_MAX_ERROR_DEGREES) {
      return 0;
    }
    lastAccepted = now;
    double error = errorSum / newFrameCount;
    if (Math.abs(error) <= ConfigVision.HEADING_ASSIST_DEADBAND_DEGREES) {
      return 0;
    }
    double maximum = ConfigVision.HEADING_ASSIST_MAX_STEP_DEGREES;
    double step = MathUtil.clamp(error * ConfigVision.HEADING_ASSIST_GAIN, -maximum, maximum);
    offsetDegrees += step;
    return step;
  }

  private static boolean slowEnough(double rate) {
    return Math.abs(rate) <= ConfigVision.HEADING_ASSIST_MAX_YAW_RATE_DEG_PER_SEC;
  }

  private static boolean hasTwoDistinctTags(VisionObservation frame) {
    if (frame.tagCount() < ConfigVision.HEADING_RESET_MEGATAG1_MIN_TAGS) {
      return false;
    }
    int[] ids = frame.tagIds();
    for (int index = 1; index < ids.length; index++) {
      if (ids[index] != ids[0]) return true;
    }
    return false;
  }
}
