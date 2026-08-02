package frc.robot.lib.swerve.characterization;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.lib.swerve.config.SwerveHardwareConfig;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/** Test-mode-only SysId and effective wheel-radius characterization commands. */
public final class SwerveCharacterization {
  private static final double WHEEL_RADIUS_TEST_OMEGA_RAD_PER_SEC = 0.50;
  private static final double WHEEL_RADIUS_TEST_SECONDS = 8.0;

  private final SwerveSubsystem drivetrain;
  private final SwerveRequest.SysIdSwerveTranslation translationRequest =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains steerRequest =
      new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation rotationRequest =
      new SwerveRequest.SysIdSwerveRotation();
  private final SwerveRequest.RobotCentric radiusRequest =
      new SwerveRequest.RobotCentric()
          .withRotationalRate(WHEEL_RADIUS_TEST_OMEGA_RAD_PER_SEC);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final double[] initialDriveRotations = new double[4];
  private double initialYawRadians;

  private final SysIdRoutine translationRoutine;
  private final SysIdRoutine steerRoutine;
  private final SysIdRoutine rotationRoutine;

  public SwerveCharacterization(SwerveSubsystem drivetrain) {
    this.drivetrain = drivetrain;
    translationRoutine =
        routine("Translation", volts -> drivetrain.setControl(translationRequest.withVolts(volts)));
    steerRoutine = routine("Steer", volts -> drivetrain.setControl(steerRequest.withVolts(volts)));
    rotationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(Math.PI / 6.0).per(Second),
                Volts.of(Math.PI),
                null,
                state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                volts ->
                    drivetrain.setControl(
                        rotationRequest.withRotationalRate(volts.in(Volts))),
                null,
                drivetrain,
                "SwerveRotation"));
  }

  private SysIdRoutine routine(
      String name, java.util.function.Consumer<edu.wpi.first.units.measure.Voltage> output) {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            null,
            state -> SignalLogger.writeString("SysId" + name + "_State", state.toString())),
        new SysIdRoutine.Mechanism(output, null, drivetrain, "Swerve" + name));
  }

  public Command translationQuasistatic(Direction direction) {
    return testOnly(translationRoutine.quasistatic(direction), "TranslationQuasistatic");
  }

  public Command translationDynamic(Direction direction) {
    return testOnly(translationRoutine.dynamic(direction), "TranslationDynamic");
  }

  public Command steerQuasistatic(Direction direction) {
    return testOnly(steerRoutine.quasistatic(direction), "SteerQuasistatic");
  }

  public Command steerDynamic(Direction direction) {
    return testOnly(steerRoutine.dynamic(direction), "SteerDynamic");
  }

  public Command rotationQuasistatic(Direction direction) {
    return testOnly(rotationRoutine.quasistatic(direction), "RotationQuasistatic");
  }

  public Command rotationDynamic(Direction direction) {
    return testOnly(rotationRoutine.dynamic(direction), "RotationDynamic");
  }

  public Command wheelRadius() {
    Command command =
        new FunctionalCommand(
                this::startWheelRadius,
                () -> drivetrain.setControl(radiusRequest),
                interrupted -> finishWheelRadius(),
                () -> false,
                drivetrain)
            .withTimeout(WHEEL_RADIUS_TEST_SECONDS)
            .withName("Swerve/WheelRadiusCharacterization");
    return testOnly(command, "WheelRadius");
  }

  private void startWheelRadius() {
    initialYawRadians = drivetrain.getPigeon2().getYaw().getValue().in(Radians);
    for (int i = 0; i < initialDriveRotations.length; i++) {
      initialDriveRotations[i] =
          drivetrain.getModule(i).getDriveMotor().getPosition().getValue().in(Rotations);
    }
  }

  private void finishWheelRadius() {
    drivetrain.setControl(brake);
    double yawDelta =
        Math.abs(drivetrain.getPigeon2().getYaw().getValue().in(Radians) - initialYawRadians);
    double averageWheelRadians = 0.0;
    for (int i = 0; i < initialDriveRotations.length; i++) {
      double rotorDelta =
          Math.abs(
              drivetrain.getModule(i).getDriveMotor().getPosition().getValue().in(Rotations)
                  - initialDriveRotations[i]);
      averageWheelRadians +=
          rotorDelta / SwerveHardwareConfig.DRIVE_RATIO.reduction() * 2.0 * Math.PI;
    }
    averageWheelRadians /= initialDriveRotations.length;

    double averageDriveRadius = 0.0;
    for (Translation2d location : drivetrain.getModuleLocations()) {
      averageDriveRadius += location.getNorm();
    }
    averageDriveRadius /= drivetrain.getModuleLocations().length;
    double effectiveRadiusMeters =
        averageWheelRadians > 1e-6 ? yawDelta * averageDriveRadius / averageWheelRadians : Double.NaN;
    SmartDashboard.putNumber("Swerve/Characterization/EffectiveWheelRadiusMeters", effectiveRadiusMeters);
    SmartDashboard.putNumber("Swerve/Characterization/YawDeltaRadians", yawDelta);
    SignalLogger.writeDouble("Swerve/Characterization/EffectiveWheelRadiusMeters", effectiveRadiusMeters);
  }

  private Command testOnly(Command command, String name) {
    return Commands.either(
            command,
            Commands.runOnce(
                () -> DriverStation.reportWarning(
                    "Caracterizacao " + name + " bloqueada: habilite Test mode.", false)),
            DriverStation::isTest)
        .withName("Swerve/TestOnly/" + name);
  }
}
