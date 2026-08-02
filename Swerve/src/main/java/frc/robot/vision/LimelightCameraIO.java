package frc.robot.vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.config.ConfigVision.CameraConfig;

/** Le MegaTag2 diretamente por NetworkTables, sem alocar a biblioteca LimelightHelpers. */
public final class LimelightCameraIO {
  private static final double[] EMPTY = new double[0];
  private static final int RAW_FIDUCIAL_STRIDE = 7;

  private final CameraConfig config;
  private final NetworkTableEntry heartbeatEntry;
  private final NetworkTableEntry targetValidEntry;
  private final NetworkTableEntry poseEntry;
  private final NetworkTableEntry rawFiducialsEntry;
  private final NetworkTableEntry primaryTagEntry;
  private final NetworkTableEntry stdDevsEntry;
  private final NetworkTableEntry orientationEntry;
  private final NetworkTableEntry cameraPoseEntry;
  private final double[] orientation = new double[6];
  private double lastHeartbeat = Double.NaN;

  public LimelightCameraIO(CameraConfig config) {
    this.config = config;
    NetworkTable table = NetworkTableInstance.getDefault().getTable(config.name());
    heartbeatEntry = table.getEntry("hb");
    targetValidEntry = table.getEntry("tv");
    poseEntry = table.getEntry("botpose_orb_wpiblue");
    rawFiducialsEntry = table.getEntry("rawfiducials");
    primaryTagEntry = table.getEntry("tid");
    stdDevsEntry = table.getEntry("stddevs");
    orientationEntry = table.getEntry("robot_orientation_set");
    cameraPoseEntry = table.getEntry("camerapose_robotspace_set");
    publishRobotSpaceCameraPose();
  }

  public CameraConfig config() {
    return config;
  }

  /** MegaTag2 exige orientacao de origem azul e velocidades angulares em graus/s. */
  public void updateRobotOrientation(
      double yawDegrees,
      double yawRateDegreesPerSecond,
      double pitchDegrees,
      double pitchRateDegreesPerSecond,
      double rollDegrees,
      double rollRateDegreesPerSecond) {
    orientation[0] = yawDegrees;
    orientation[1] = yawRateDegreesPerSecond;
    orientation[2] = pitchDegrees;
    orientation[3] = pitchRateDegreesPerSecond;
    orientation[4] = rollDegrees;
    orientation[5] = rollRateDegreesPerSecond;
    orientationEntry.setDoubleArray(orientation);
  }

  /** Retorna apenas frames novos para nao fundir repetidamente a mesma imagem. */
  public Optional<VisionObservation> readLatestObservation() {
    double heartbeat = heartbeatEntry.getDouble(Double.NaN);
    if (Double.isFinite(heartbeat) && heartbeat == lastHeartbeat) {
      return Optional.empty();
    }
    lastHeartbeat = heartbeat;

    if (targetValidEntry.getDouble(0.0) != 1.0) {
      return Optional.empty();
    }

    double[] pose = poseEntry.getDoubleArray(EMPTY);
    if (pose.length < 11) {
      return Optional.empty();
    }

    double latencySeconds = Math.max(0.0, pose[6]) / 1000.0;
    double timestampSeconds = Timer.getFPGATimestamp() - latencySeconds;
    int reportedTagCount = Math.max(0, (int) Math.round(pose[7]));

    double[] rawFiducials = rawFiducialsEntry.getDoubleArray(EMPTY);
    int parsedCount = rawFiducials.length / RAW_FIDUCIAL_STRIDE;
    int[] tagIds;
    double maxAmbiguity = 0.0;
    if (parsedCount > 0) {
      tagIds = new int[parsedCount];
      for (int i = 0; i < parsedCount; i++) {
        int offset = i * RAW_FIDUCIAL_STRIDE;
        tagIds[i] = (int) Math.round(rawFiducials[offset]);
        maxAmbiguity = Math.max(maxAmbiguity, rawFiducials[offset + 6]);
      }
    } else {
      int primaryId = (int) Math.round(primaryTagEntry.getDouble(-1));
      tagIds = primaryId > 0 ? new int[] {primaryId} : new int[0];
    }

    double[] stdDevs = stdDevsEntry.getDoubleArray(EMPTY);
    double reportedXStdDev = stdDevs.length >= 8 ? stdDevs[6] : Double.NaN;
    double reportedYStdDev = stdDevs.length >= 8 ? stdDevs[7] : Double.NaN;

    Pose3d robotPose =
        new Pose3d(
            pose[0],
            pose[1],
            pose[2],
            new Rotation3d(
                Units.degreesToRadians(pose[3]),
                Units.degreesToRadians(pose[4]),
                Units.degreesToRadians(pose[5])));

    return Optional.of(
        new VisionObservation(
            config.name(),
            robotPose,
            timestampSeconds,
            tagIds,
            reportedTagCount,
            pose[9],
            pose[10],
            maxAmbiguity,
            reportedXStdDev,
            reportedYStdDev));
  }

  private void publishRobotSpaceCameraPose() {
    Transform3d transform = config.robotToCamera();
    cameraPoseEntry.setDoubleArray(
            new double[] {
              transform.getX(),
              -transform.getY(), // Limelight documenta este eixo como "right".
              transform.getZ(),
              Units.radiansToDegrees(transform.getRotation().getX()),
              Units.radiansToDegrees(transform.getRotation().getY()),
              Units.radiansToDegrees(transform.getRotation().getZ())
            });
  }
}
