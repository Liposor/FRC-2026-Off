package frc.robot.lib.swerve.config;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;

/** Single source of truth for the known mechanical and electrical configuration. */
public final class SwerveHardwareConfig {
  private SwerveHardwareConfig() {}

  public enum Mk5nRatio {
    R1(7.03, 14.4),
    R2(6.03, 16.8),
    R3(5.27, 19.2);

    private final double reduction;
    private final double freeSpeedFeetPerSecond;

    Mk5nRatio(double reduction, double freeSpeedFeetPerSecond) {
      this.reduction = reduction;
      this.freeSpeedFeetPerSecond = freeSpeedFeetPerSecond;
    }

    public double reduction() {
      return reduction;
    }

    public LinearVelocity theoreticalFreeSpeed() {
      return MetersPerSecond.of(freeSpeedFeetPerSecond * 0.3048);
    }
  }

  public static final Mk5nRatio DRIVE_RATIO = Mk5nRatio.R3;
  public static final double STEER_RATIO = 287.0 / 11.0;

  /** 54/16 is the inverse of the first drive stage. Confirm in Tuner X on the real module. */
  public static final double COUPLING_RATIO = 54.0 / 16.0;

  public static final Distance NOMINAL_WHEEL_RADIUS = Inches.of(2.0);
  /** Change only after wheel-radius characterization, never from nominal CAD alone. */
  public static final double EFFECTIVE_WHEEL_RADIUS_SCALE = 1.0;
  public static final Distance EFFECTIVE_WHEEL_RADIUS =
      Inches.of(NOMINAL_WHEEL_RADIUS.in(Inches) * EFFECTIVE_WHEEL_RADIUS_SCALE);

  public static final Mass ROBOT_MASS = Kilograms.of(51.79);
  public static final Voltage NORMAL_STARTING_BATTERY = Volts.of(12.3);
  public static final Voltage BEST_MEASURED_BATTERY = Volts.of(12.7);

  public static final String DRIVE_MOTOR = "Kraken X60";
  public static final String STEER_MOTOR = "Kraken X44";
  public static final String ABSOLUTE_ENCODER = "CANcoder";
  public static final String GYRO = "Pigeon 2.0";
  public static final String POWER_DISTRIBUTION = "REV PDH";
  public static final int PDH_CAN_ID = 1; // Placeholder until confirmed.

  /** `*` selects the only CANivore without depending on its user-facing name. */
  public static String canBusName() {
    return RobotBase.isReal() ? "*" : "rio";
  }

  /* Safe simulation placeholders. Real operation remains blocked until all are measured. */
  public static final Distance HALF_WHEELBASE = Inches.of(10.0);
  public static final Distance HALF_TRACKWIDTH = Inches.of(10.0);
  public static final Distance BUMPER_LENGTH = Inches.of(30.0);
  public static final Distance BUMPER_WIDTH = Inches.of(30.0);
  public static final double PROVISIONAL_WHEEL_COEFFICIENT_OF_FRICTION = 1.20;
  public static final boolean BUMPER_DIMENSIONS_MEASURED = false;
  public static final boolean MODULE_GEOMETRY_MEASURED = false;
  public static final boolean CAN_IDS_CONFIRMED = false;
  public static final boolean CANCODER_OFFSETS_CALIBRATED = false;
  public static final boolean CAMERA_TRANSFORMS_MEASURED = false;
  public static final boolean CURRENT_LIMITS_VALIDATED = false;
  public static final boolean GAINS_CHARACTERIZED = false;
  public static final boolean COUPLING_RATIO_CONFIRMED = false;
  public static final boolean PHOENIX_PRO_LICENSE_CONFIRMED = false;

  public static boolean isReadyForRealHardware() {
    return MODULE_GEOMETRY_MEASURED
        && BUMPER_DIMENSIONS_MEASURED
        && CAN_IDS_CONFIRMED
        && CANCODER_OFFSETS_CALIBRATED
        && CAMERA_TRANSFORMS_MEASURED
        && CURRENT_LIMITS_VALIDATED
        && GAINS_CHARACTERIZED
        && COUPLING_RATIO_CONFIRMED;
  }

  public static List<String> missingRealHardwareData() {
    var missing = new java.util.ArrayList<String>();
    if (!MODULE_GEOMETRY_MEASURED) missing.add("distancias X/Y entre centros dos modulos");
    if (!BUMPER_DIMENSIONS_MEASURED) missing.add("comprimento e largura externos dos bumpers");
    if (!CAN_IDS_CONFIRMED) missing.add("IDs CAN de motores, CANcoders, Pigeon e PDH");
    if (!CANCODER_OFFSETS_CALIBRATED) missing.add("offset absoluto de cada CANcoder");
    if (!CAMERA_TRANSFORMS_MEASURED) missing.add("transform 3D das tres Limelight");
    if (!CURRENT_LIMITS_VALIDATED) missing.add("limites de corrente e corrente de slip");
    if (!GAINS_CHARACTERIZED) missing.add("ganhos de drive/steer por SysId e testes");
    if (!COUPLING_RATIO_CONFIRMED) missing.add("coupling ratio 54/16 no Tuner X");
    return List.copyOf(missing);
  }
}
