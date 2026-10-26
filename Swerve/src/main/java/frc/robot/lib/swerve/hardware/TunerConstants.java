package frc.robot.lib.swerve.hardware;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.lib.swerve.config.ConfigSwerve;
import frc.robot.lib.swerve.config.SwerveHardwareConfig;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/**
 * Configuracao inicial do swerve para simulacao.
 *
 * <p>Os valores deste arquivo sao exemplos coerentes, baseados no projeto gerado pelo
 * Tuner X 2026. Gere/substitua estes valores no Tuner X antes de executar no robo real.
 */
public final class TunerConstants {
  private TunerConstants() {}

  /** Troque para true somente depois de configurar e validar o hardware no Tuner X. */
  public static final boolean HARDWARE_CONFIGURED =
      SwerveHardwareConfig.isReadyForRealHardware();

  private static final Slot0Configs STEER_GAINS =
      new Slot0Configs()
          .withKP(100)
          .withKI(0)
          .withKD(0.5)
          .withKS(0.1)
          .withKV(1.91)
          .withKA(0)
          .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

  private static final Slot0Configs DRIVE_GAINS =
      new Slot0Configs().withKP(0.1).withKI(0).withKD(0).withKS(0).withKV(0.124);

  private static final Current SLIP_CURRENT = Amps.of(120);

  private static final TalonFXConfiguration DRIVE_INITIAL_CONFIGS =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withSupplyCurrentLimit(Amps.of(70))
                  .withSupplyCurrentLimitEnable(true));

  private static final TalonFXConfiguration STEER_INITIAL_CONFIGS =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(Amps.of(60))
                  .withStatorCurrentLimitEnable(true));

  private static final CANcoderConfiguration ENCODER_INITIAL_CONFIGS =
      new CANcoderConfiguration();
  private static final Pigeon2Configuration PIGEON_CONFIGS = null;

  /** Todos os dispositivos do swerve precisam estar no mesmo barramento CAN. */
  public static final CANBus CAN_BUS =
      new CANBus(SwerveHardwareConfig.canBusName(), "./logs/swerve.hoot");

  // Valores mecanicos de exemplo. Substitua pelos valores do seu modulo.
  private static final double COUPLING_GEAR_RATIO = SwerveHardwareConfig.COUPLING_RATIO;
  private static final double DRIVE_GEAR_RATIO = SwerveHardwareConfig.DRIVE_RATIO.reduction();
  private static final double STEER_GEAR_RATIO = SwerveHardwareConfig.STEER_RATIO;
  private static final Distance WHEEL_RADIUS = SwerveHardwareConfig.EFFECTIVE_WHEEL_RADIUS;

  private static final boolean INVERT_LEFT_SIDE = false;
  private static final boolean INVERT_RIGHT_SIDE = true;
  private static final int PIGEON_ID = 13;

  // Parametros usados pelo modelo de simulacao da Phoenix 6.
  private static final MomentOfInertia STEER_INERTIA = KilogramSquareMeters.of(0.01);
  private static final MomentOfInertia DRIVE_INERTIA = KilogramSquareMeters.of(0.035);
  private static final Voltage STEER_FRICTION_VOLTAGE = Volts.of(0.2);
  private static final Voltage DRIVE_FRICTION_VOLTAGE = Volts.of(0.2);

  public static final SwerveDrivetrainConstants DRIVETRAIN_CONSTANTS =
      new SwerveDrivetrainConstants()
          .withCANBusName(CAN_BUS.getName())
          .withPigeon2Id(PIGEON_ID)
          .withPigeon2Configs(PIGEON_CONFIGS);

  private static final
      SwerveModuleConstantsFactory<
              TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          MODULE_FACTORY =
              new SwerveModuleConstantsFactory<
                      TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                  .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
                  .withSteerMotorGearRatio(STEER_GEAR_RATIO)
                  .withCouplingGearRatio(COUPLING_GEAR_RATIO)
                  .withWheelRadius(WHEEL_RADIUS)
                  .withSteerMotorGains(STEER_GAINS)
                  .withDriveMotorGains(DRIVE_GAINS)
                  .withSteerMotorClosedLoopOutput(
                      SwerveModuleConstants.ClosedLoopOutputType.Voltage)
                  .withDriveMotorClosedLoopOutput(
                      SwerveModuleConstants.ClosedLoopOutputType.Voltage)
                  .withSlipCurrent(SLIP_CURRENT)
                  .withSpeedAt12Volts(ConfigSwerve.SPEED_AT_12_VOLTS)
                  .withDriveMotorType(DriveMotorArrangement.TalonFX_Integrated)
                  .withSteerMotorType(SteerMotorArrangement.TalonFX_Integrated)
                  .withFeedbackSource(
                      SwerveHardwareConfig.PHOENIX_PRO_LICENSE_CONFIRMED
                          ? SteerFeedbackType.FusedCANcoder
                          : SteerFeedbackType.RemoteCANcoder)
                  .withDriveMotorInitialConfigs(DRIVE_INITIAL_CONFIGS)
                  .withSteerMotorInitialConfigs(STEER_INITIAL_CONFIGS)
                  .withEncoderInitialConfigs(ENCODER_INITIAL_CONFIGS)
                  .withSteerInertia(STEER_INERTIA)
                  .withDriveInertia(DRIVE_INERTIA)
                  .withSteerFrictionVoltage(STEER_FRICTION_VOLTAGE)
                  .withDriveFrictionVoltage(DRIVE_FRICTION_VOLTAGE);

  private static final Distance HALF_WHEELBASE = SwerveHardwareConfig.HALF_WHEELBASE;
  private static final Distance HALF_TRACKWIDTH = SwerveHardwareConfig.HALF_TRACKWIDTH;
  private static final Distance NEGATIVE_HALF_WHEELBASE =
      Inches.of(-HALF_WHEELBASE.in(Inches));
  private static final Distance NEGATIVE_HALF_TRACKWIDTH =
      Inches.of(-HALF_TRACKWIDTH.in(Inches));

  // IDs e offsets de exemplo para simulacao.
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FRONT_LEFT =
          createModule(2, 3, 1, 0.15234375, HALF_WHEELBASE, HALF_TRACKWIDTH, false);

  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FRONT_RIGHT =
          createModule(5, 6, 4, -0.4873046875, HALF_WHEELBASE, NEGATIVE_HALF_TRACKWIDTH, true);

  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BACK_LEFT =
          createModule(8, 9, 7, -0.219482421875, NEGATIVE_HALF_WHEELBASE, HALF_TRACKWIDTH, false);

  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BACK_RIGHT =
          createModule(
              11,
              12,
              10,
              0.17236328125,
              NEGATIVE_HALF_WHEELBASE,
              NEGATIVE_HALF_TRACKWIDTH,
              true);

  private static SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      createModule(
          int steerId,
          int driveId,
          int encoderId,
          double encoderOffsetRotations,
          Distance x,
          Distance y,
          boolean rightSide) {
    return MODULE_FACTORY.createModuleConstants(
        steerId,
        driveId,
        encoderId,
        Rotations.of(encoderOffsetRotations),
        x,
        y,
        rightSide ? INVERT_RIGHT_SIDE : INVERT_LEFT_SIDE,
        true,
        false);
  }

  public static SwerveSubsystem createDrivetrain() {
    if (edu.wpi.first.wpilibj.RobotBase.isReal() && !HARDWARE_CONFIGURED) {
      throw new IllegalStateException(
          "Swerve bloqueado no robo real. Faltam: "
              + String.join(", ", SwerveHardwareConfig.missingRealHardwareData()));
    }
    return new SwerveSubsystem(
        DRIVETRAIN_CONSTANTS, FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT);
  }

  /** Drivetrain tipado para Talon FX + CANcoder. */
  public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
    public TunerSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules) {
      super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, modules);
    }
  }
}
