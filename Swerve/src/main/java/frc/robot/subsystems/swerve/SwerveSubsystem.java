package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Seconds;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.lib.swerve.config.ConfigLocalization;
import frc.robot.lib.swerve.config.ConfigSwerve;
import frc.robot.lib.swerve.config.SwerveHardwareConfig;
import frc.robot.lib.swerve.hardware.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.lib.swerve.simulation.MapleSimSwerveDrivetrain;

/** Integra o drivetrain Phoenix 6 ao framework Command-Based e a simulacao desktop. */
public class SwerveSubsystem extends TunerSwerveDrivetrain implements Subsystem {
  private static final Rotation2d BLUE_PERSPECTIVE = Rotation2d.kZero;
  private static final Rotation2d RED_PERSPECTIVE = Rotation2d.k180deg;

  private Notifier simNotifier;
  private MapleSimSwerveDrivetrain mapleSimulation;
  private boolean hasAppliedOperatorPerspective;
  private boolean pathPlannerConfigured;
  private RobotConfig pathPlannerRobotConfig;
  private BooleanSupplier pathMotionBlocked = () -> false;
  private Consumer<ChassisSpeeds> requestedSpeedsObserver = speeds -> {};
  private final SwerveRequest.ApplyRobotSpeeds pathPlannerRequest =
      new SwerveRequest.ApplyRobotSpeeds().withDriveRequestType(DriveRequestType.Velocity);
  private final SwerveRequest.SwerveDriveBrake pathPlannerBrake =
      new SwerveRequest.SwerveDriveBrake();

  public SwerveSubsystem(
      SwerveDrivetrainConstants drivetrainConstants,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, MapleSimSwerveDrivetrain.regulateForSimulation(modules));

    configurePathPlanner();

    if (Utils.isSimulation()) {
      startSimThread(modules);
    }
  }

  /** Cria um comando que atualiza continuamente uma requisicao nativa do swerve. */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> setControl(requestSupplier.get()));
  }

  public boolean isPathPlannerConfigured() {
    return pathPlannerConfigured;
  }

  public RobotConfig getPathPlannerRobotConfig() {
    return pathPlannerRobotConfig;
  }

  public void setPathMotionBlockedSupplier(BooleanSupplier blockedSupplier) {
    pathMotionBlocked = blockedSupplier != null ? blockedSupplier : () -> false;
  }

  public void setRequestedSpeedsObserver(Consumer<ChassisSpeeds> observer) {
    requestedSpeedsObserver = observer != null ? observer : speeds -> {};
  }

  public void observeRequestedSpeeds(ChassisSpeeds speeds) {
    requestedSpeedsObserver.accept(speeds);
  }

  /** Converte o relogio FPGA usado pela Limelight para o relogio interno do Phoenix. */
  @Override
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    super.addVisionMeasurement(
        visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
  }

  @Override
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    super.addVisionMeasurement(
        visionRobotPoseMeters,
        Utils.fpgaToCurrentTime(timestampSeconds),
        visionMeasurementStdDevs);
  }

  @Override
  public void periodic() {
    // A perspectiva do piloto so muda enquanto desabilitado, evitando inversao durante a partida.
    if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              alliance -> {
                setOperatorPerspectiveForward(
                    alliance == Alliance.Red ? RED_PERSPECTIVE : BLUE_PERSPECTIVE);
                hasAppliedOperatorPerspective = true;
              });
    }
  }

  private void startSimThread(SwerveModuleConstants<?, ?, ?>[] modules) {
    mapleSimulation =
        new MapleSimSwerveDrivetrain(
            Seconds.of(ConfigSwerve.SIM_LOOP_PERIOD_SECONDS),
            SwerveHardwareConfig.ROBOT_MASS,
            SwerveHardwareConfig.BUMPER_LENGTH,
            SwerveHardwareConfig.BUMPER_WIDTH,
            DCMotor.getKrakenX60Foc(1),
            DCMotor.getKrakenX44Foc(1),
            SwerveHardwareConfig.PROVISIONAL_WHEEL_COEFFICIENT_OF_FRICTION,
            getModuleLocations(),
            getPigeon2(),
            getModules(),
            modules);
    simNotifier =
        new Notifier(
            mapleSimulation::update);
    simNotifier.setName("MapleSimSwerve");
    simNotifier.startPeriodic(ConfigSwerve.SIM_LOOP_PERIOD_SECONDS);
  }

  private void configurePathPlanner() {
    try {
      RobotConfig robotConfig = RobotConfig.fromGUISettings();
      pathPlannerRobotConfig = robotConfig;
      AutoBuilder.configure(
          () -> getState().Pose,
          this::resetPose,
          () -> getState().Speeds,
          (speeds, feedforwards) -> {
            requestedSpeedsObserver.accept(speeds);
            if (pathMotionBlocked.getAsBoolean()) {
              setControl(pathPlannerBrake);
            } else {
              setControl(
                  pathPlannerRequest
                      .withSpeeds(speeds)
                      .withWheelForceFeedforwardsX(
                          feedforwards.robotRelativeForcesXNewtons())
                      .withWheelForceFeedforwardsY(
                          feedforwards.robotRelativeForcesYNewtons()));
            }
          },
          new PPHolonomicDriveController(
              new PIDConstants(ConfigLocalization.PATH_TRANSLATION_KP, 0.0, 0.0),
              new PIDConstants(ConfigLocalization.PATH_ROTATION_KP, 0.0, 0.0)),
          robotConfig,
          () ->
              DriverStation.getAlliance()
                  .map(alliance -> alliance == Alliance.Red)
                  .orElse(false),
          this);
      pathPlannerConfigured = true;
    } catch (Exception exception) {
      pathPlannerConfigured = false;
      pathPlannerRobotConfig = null;
      DriverStation.reportError(
          "PathPlanner nao configurado: " + exception.getMessage(),
          exception.getStackTrace());
    }
  }
}
