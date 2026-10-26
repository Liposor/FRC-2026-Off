package frc.robot.lib.swerve.simulation;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Dashboard-controlled deterministic faults for autonomous and estimator testing in simulation. */
public final class SimulationFaults {
  private static final NetworkTable TABLE =
      NetworkTableInstance.getDefault().getTable("SimulationFaults");
  private static final NetworkTableEntry VISION_DROPOUT = TABLE.getEntry("VisionDropoutAll");
  private static final NetworkTableEntry VISION_OUTLIER_X = TABLE.getEntry("VisionOutlierXMeters");
  private static final NetworkTableEntry EXTRA_VISION_LATENCY =
      TABLE.getEntry("ExtraVisionLatencyMilliseconds");
  private static final NetworkTableEntry CANCODER_OFFSET =
      TABLE.getEntry("CANcoderOffsetRotations");

  static {
    VISION_DROPOUT.setDefaultBoolean(false);
    VISION_OUTLIER_X.setDefaultDouble(0.0);
    EXTRA_VISION_LATENCY.setDefaultDouble(0.0);
    CANCODER_OFFSET.setDefaultDouble(0.0);
  }

  private SimulationFaults() {}

  public static boolean visionDropout() {
    return VISION_DROPOUT.getBoolean(false);
  }

  public static double visionOutlierXMeters() {
    return VISION_OUTLIER_X.getDouble(0.0);
  }

  public static double extraVisionLatencySeconds() {
    return Math.max(0.0, EXTRA_VISION_LATENCY.getDouble(0.0)) / 1000.0;
  }

  public static double cancoderOffsetRotations() {
    return CANCODER_OFFSET.getDouble(0.0);
  }
}
