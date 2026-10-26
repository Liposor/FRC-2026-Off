package frc.robot.lib.swerve.state;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.swerve.config.ConfigLocalization;

/** Central, timestamped view of robot motion and high-level state with fixed memory use. */
public final class RobotState extends SubsystemBase {
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> speedsSupplier;
  private final Supplier<String> driveHealthSupplier;
  private final Supplier<String> superstructureSupplier;
  private final Sample[] history = new Sample[ConfigLocalization.HISTORY_CAPACITY];
  private final StructPublisher<Pose2d> posePublisher =
      NetworkTableInstance.getDefault()
          .getStructTopic("RobotState/Pose", Pose2d.struct)
          .publish();
  private final StructPublisher<ChassisSpeeds> speedsPublisher =
      NetworkTableInstance.getDefault()
          .getStructTopic("RobotState/RobotRelativeSpeeds", ChassisSpeeds.struct)
          .publish();
  private final StringPublisher healthPublisher =
      NetworkTableInstance.getDefault().getStringTopic("RobotState/DriveHealth").publish();
  private final StringPublisher goalPublisher =
      NetworkTableInstance.getDefault().getStringTopic("RobotState/SuperstructureGoal").publish();
  private int nextIndex;
  private int size;
  private double lastSampleSeconds = Double.NEGATIVE_INFINITY;
  private double lastPublishSeconds = Double.NEGATIVE_INFINITY;

  public RobotState(
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> speedsSupplier,
      Supplier<String> driveHealthSupplier,
      Supplier<String> superstructureSupplier) {
    this.poseSupplier = poseSupplier;
    this.speedsSupplier = speedsSupplier;
    this.driveHealthSupplier = driveHealthSupplier;
    this.superstructureSupplier = superstructureSupplier;
    for (int i = 0; i < history.length; i++) history[i] = new Sample();
  }

  @Override
  public void periodic() {
    double now = Timer.getFPGATimestamp();
    if (now - lastSampleSeconds >= ConfigLocalization.SAMPLE_PERIOD_SECONDS) {
      lastSampleSeconds = now;
      Pose2d pose = poseSupplier.get();
      ChassisSpeeds speeds = speedsSupplier.get();
      Sample sample = history[nextIndex];
      sample.timestamp = now;
      sample.pose = pose;
      sample.speeds =
          new ChassisSpeeds(
              speeds.vxMetersPerSecond,
              speeds.vyMetersPerSecond,
              speeds.omegaRadiansPerSecond);
      nextIndex = (nextIndex + 1) % history.length;
      size = Math.min(size + 1, history.length);
    }
    if (now - lastPublishSeconds >= 0.05) {
      lastPublishSeconds = now;
      posePublisher.set(poseSupplier.get());
      speedsPublisher.set(speedsSupplier.get());
      healthPublisher.set(driveHealthSupplier.get());
      goalPublisher.set(superstructureSupplier.get());
    }
  }

  /** Returns the nearest stored sample; intended for latency diagnostics and event review. */
  public Optional<StateSnapshot> nearestSample(double timestampSeconds) {
    if (size == 0) return Optional.empty();
    Sample best = null;
    double bestDelta = Double.POSITIVE_INFINITY;
    for (int step = 0; step < size; step++) {
      Sample sample = history[Math.floorMod(nextIndex - 1 - step, history.length)];
      double delta = Math.abs(sample.timestamp - timestampSeconds);
      if (delta < bestDelta) {
        bestDelta = delta;
        best = sample;
      }
    }
    return Optional.of(new StateSnapshot(best.timestamp, best.pose, best.speeds));
  }

  public record StateSnapshot(double timestampSeconds, Pose2d pose, ChassisSpeeds speeds) {}

  private static final class Sample {
    double timestamp;
    Pose2d pose = Pose2d.kZero;
    ChassisSpeeds speeds = new ChassisSpeeds();
  }
}
