package frc.robot.lib.swerve.vision;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.lib.swerve.config.ConfigVision;

/** Combines simultaneous camera estimates without pretending correlated frames are independent. */
public final class VisionConsensus {
  private VisionConsensus() {}

  public record Candidate(
      String camera,
      Pose2d pose,
      double timestampSeconds,
      double xyStdDevMeters,
      double confidence) {
    public Candidate {
      if (camera == null || camera.isBlank() || pose == null) {
        throw new IllegalArgumentException("Candidate requires camera and pose");
      }
    }
  }

  public record Result(
      boolean accepted,
      Pose2d pose,
      double timestampSeconds,
      double xyStdDevMeters,
      int supportingCameras,
      double maxResidualMeters,
      String reason) {
    public static Result none(String reason) {
      return new Result(false, Pose2d.kZero, 0.0, Double.POSITIVE_INFINITY, 0, 0.0, reason);
    }
  }

  public static Result solve(List<Candidate> candidates) {
    if (candidates == null || candidates.isEmpty()) {
      return Result.none("NO_ACCEPTED_CAMERA");
    }
    if (candidates.size() == 1) {
      Candidate only = candidates.get(0);
      return new Result(
          true,
          only.pose(),
          only.timestampSeconds(),
          only.xyStdDevMeters() * ConfigVision.SINGLE_CAMERA_STD_DEV_MULTIPLIER,
          1,
          0.0,
          "SINGLE_CAMERA_INFLATED");
    }

    List<Candidate> ordered =
        candidates.stream()
            .sorted(Comparator.comparingDouble(Candidate::confidence).reversed())
            .toList();
    List<Candidate> bestCluster = List.of();
    for (Candidate anchor : ordered) {
      List<Candidate> cluster = new ArrayList<>();
      for (Candidate candidate : ordered) {
        boolean closeInTime =
            Math.abs(candidate.timestampSeconds() - anchor.timestampSeconds())
                <= ConfigVision.CONSENSUS_MAX_TIMESTAMP_DIFFERENCE_SECONDS;
        boolean closeInSpace =
            candidate.pose().getTranslation().getDistance(anchor.pose().getTranslation())
                <= ConfigVision.CONSENSUS_MAX_TRANSLATION_METERS;
        if (closeInTime && closeInSpace) cluster.add(candidate);
      }
      if (cluster.size() > bestCluster.size()) bestCluster = List.copyOf(cluster);
    }

    if (bestCluster.size() < 2) {
      return Result.none("CAMERA_DISAGREEMENT");
    }

    double weightSum = 0.0;
    double x = 0.0;
    double y = 0.0;
    double timestamp = 0.0;
    double baseStdDev = Double.POSITIVE_INFINITY;
    for (Candidate candidate : bestCluster) {
      double variance = Math.max(1e-6, candidate.xyStdDevMeters() * candidate.xyStdDevMeters());
      double weight = MathUtil.clamp(candidate.confidence(), 0.05, 1.0) / variance;
      weightSum += weight;
      x += candidate.pose().getX() * weight;
      y += candidate.pose().getY() * weight;
      timestamp += candidate.timestampSeconds() * weight;
      baseStdDev = Math.min(baseStdDev, candidate.xyStdDevMeters());
    }
    x /= weightSum;
    y /= weightSum;
    timestamp /= weightSum;

    Pose2d pose = new Pose2d(x, y, Rotation2d.kZero);
    double maxResidual = 0.0;
    for (Candidate candidate : bestCluster) {
      maxResidual =
          Math.max(
              maxResidual,
              pose.getTranslation().getDistance(candidate.pose().getTranslation()));
    }
    return new Result(
        true,
        pose,
        timestamp,
        baseStdDev * ConfigVision.MULTI_CAMERA_STD_DEV_MULTIPLIER,
        bestCluster.size(),
        maxResidual,
        "CONSENSUS");
  }
}
