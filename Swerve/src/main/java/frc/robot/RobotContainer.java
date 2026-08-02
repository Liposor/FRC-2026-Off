package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.config.ConfigSwerve;
import frc.robot.config.ConfigSwerve.DriveProfile;
import frc.robot.generated.TunerConstants;
import frc.robot.localization.OdometryHealthMonitor;
import frc.robot.localization.SwerveStateSolver;
import frc.robot.localization.SwerveStateSolver.DriveState;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Goal;
import frc.robot.util.PerformanceMonitor;
import frc.robot.vision.LimelightSimulation;
import frc.robot.vision.VisionSubsystem;

public class RobotContainer {
  private final double maxSpeedMetersPerSecond =
      ConfigSwerve.TELEOP_MAX_SPEED.in(MetersPerSecond);
  private final double maxAngularRateRadiansPerSecond =
      ConfigSwerve.TELEOP_MAX_ANGULAR_RATE.in(RadiansPerSecond);

  private final CommandXboxController driverController = new CommandXboxController(0);

  private final SwerveRequest.FieldCentric fieldCentricDrive =
      new SwerveRequest.FieldCentric()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.SwerveDriveBrake brake =
      new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt pointWheels =
      new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.Idle idle = new SwerveRequest.Idle();

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  public final Superstructure superstructure = new Superstructure();
  public final OdometryHealthMonitor odometryHealth = new OdometryHealthMonitor(drivetrain);
  public final LimelightSimulation limelightSimulation =
      new LimelightSimulation(() -> drivetrain.getState().Pose);
  public final VisionSubsystem vision = new VisionSubsystem(drivetrain);
  public final PerformanceMonitor performanceMonitor = new PerformanceMonitor();

  private final SwerveStateSolver swerveStateSolver = new SwerveStateSolver();
  private DriveState lastPublishedDriveState;
  private final Telemetry telemetry = new Telemetry();
  private final SendableChooser<Command> autonomousChooser;

  public RobotContainer() {
    drivetrain.setPathMotionBlockedSupplier(odometryHealth::shouldHoldPosition);
    drivetrain.setRequestedSpeedsObserver(odometryHealth::setRequestedSpeeds);
    configureBindings();
    autonomousChooser = configureAutonomousChooser();
    drivetrain.registerTelemetry(telemetry::telemeterize);

    SmartDashboard.putBoolean(
        "Swerve/HardwareConfigured", TunerConstants.HARDWARE_CONFIGURED);
  }

  private void configureBindings() {
    // Convencao WPILib: +X para frente, +Y para a esquerda e giro anti-horario positivo.
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () -> {
              DriveProfile profile = superstructure.getDriveProfile();
              double velocityX =
                  -MathUtil.applyDeadband(
                          driverController.getLeftY(), ConfigSwerve.JOYSTICK_DEADBAND)
                      * maxSpeedMetersPerSecond
                      * profile.translationScale();
              double velocityY =
                  -MathUtil.applyDeadband(
                          driverController.getLeftX(), ConfigSwerve.JOYSTICK_DEADBAND)
                      * maxSpeedMetersPerSecond
                      * profile.translationScale();
              double rotationalRate =
                  -MathUtil.applyDeadband(
                          driverController.getRightX(), ConfigSwerve.JOYSTICK_DEADBAND)
                      * maxAngularRateRadiansPerSecond
                      * profile.rotationScale();

              odometryHealth.setRequestedSpeeds(velocityX, velocityY, rotationalRate);
              DriveState driveState =
                  swerveStateSolver.solve(
                      superstructure.getGoal(), odometryHealth.shouldHoldPosition());
              if (driveState != lastPublishedDriveState) {
                lastPublishedDriveState = driveState;
                SmartDashboard.putString("Swerve/SolverState", driveState.name());
              }
              if (driveState != DriveState.NORMAL) {
                return brake;
              }

              return fieldCentricDrive
                  .withVelocityX(velocityX)
                  .withVelocityY(velocityY)
                  .withRotationalRate(rotationalRate);
            }));

    RobotModeTriggers.disabled()
        .whileTrue(
            drivetrain
                .applyRequest(() -> idle)
                .ignoringDisable(true));

    driverController
        .a()
        .whileTrue(
            drivetrain.applyRequest(
                () -> {
                  odometryHealth.setRequestedSpeeds(0.0, 0.0, 0.0);
                  return brake;
                }));
    driverController.b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    pointWheels.withModuleDirection(
                        new Rotation2d(
                            -driverController.getLeftY(), -driverController.getLeftX()))));
    driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    driverController
        .back()
        .onTrue(
            drivetrain.runOnce(
                () -> {
                  boolean applied = odometryHealth.applyRecommendedRecoveryPose();
                  SmartDashboard.putBoolean("OdometryHealth/LastManualRecoveryApplied", applied);
                }));

    // Goals temporarios para testar a Superstructure sem mecanismos instalados.
    driverController
        .rightTrigger()
        .whileTrue(superstructure.holdGoalCommand(Goal.COLLECTING, Goal.HOLDING_GAME_PIECE));
    driverController
        .leftTrigger()
        .whileTrue(superstructure.holdGoalCommand(Goal.SCORING, Goal.IDLE));
    driverController.y().onTrue(superstructure.setGoalCommand(Goal.CLIMBING));
    driverController.x().onTrue(superstructure.setGoalCommand(Goal.IDLE));
    driverController
        .rightBumper()
        .onTrue(superstructure.setGoalCommand(Goal.HOLDING_GAME_PIECE));
  }

  private SendableChooser<Command> configureAutonomousChooser() {
    SendableChooser<Command> chooser =
        drivetrain.isPathPlannerConfigured()
            ? AutoBuilder.buildAutoChooser()
            : new SendableChooser<>();
    chooser.setDefaultOption("Parado", Commands.none());
    chooser.addOption("Reto - 2 segundos", createDriveStraightAuto());
    chooser.addOption("Quadrado - simulacao", createSquareAuto());
    SmartDashboard.putData("Autonomo", chooser);
    SmartDashboard.putBoolean(
        "PathPlanner/Configured", drivetrain.isPathPlannerConfigured());
    return chooser;
  }

  private Command createDriveStraightAuto() {
    return Commands.sequence(
        resetSimulatedPose(),
        driveRobotRelative(
            ConfigSwerve.AUTO_TRANSLATION_SPEED_METERS_PER_SECOND,
            0.0,
            0.0,
            ConfigSwerve.AUTO_STRAIGHT_DURATION_SECONDS),
        holdBrake(ConfigSwerve.AUTO_BRAKE_DURATION_SECONDS));
  }

  private Command createSquareAuto() {
    return Commands.sequence(
        resetSimulatedPose(),
        driveRobotRelative(
            ConfigSwerve.AUTO_TRANSLATION_SPEED_METERS_PER_SECOND,
            0.0,
            0.0,
            ConfigSwerve.AUTO_SQUARE_SIDE_DURATION_SECONDS),
        driveRobotRelative(
            0.0,
            ConfigSwerve.AUTO_TRANSLATION_SPEED_METERS_PER_SECOND,
            0.0,
            ConfigSwerve.AUTO_SQUARE_SIDE_DURATION_SECONDS),
        driveRobotRelative(
            -ConfigSwerve.AUTO_TRANSLATION_SPEED_METERS_PER_SECOND,
            0.0,
            0.0,
            ConfigSwerve.AUTO_SQUARE_SIDE_DURATION_SECONDS),
        driveRobotRelative(
            0.0,
            -ConfigSwerve.AUTO_TRANSLATION_SPEED_METERS_PER_SECOND,
            0.0,
            ConfigSwerve.AUTO_SQUARE_SIDE_DURATION_SECONDS),
        holdBrake(ConfigSwerve.AUTO_BRAKE_DURATION_SECONDS));
  }

  private Command resetSimulatedPose() {
    return drivetrain.runOnce(() -> drivetrain.resetPose(new Pose2d()));
  }

  private Command driveRobotRelative(
      double velocityX, double velocityY, double rotationalRate, double seconds) {
    SwerveRequest.RobotCentric request =
        new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withVelocityX(velocityX)
            .withVelocityY(velocityY)
            .withRotationalRate(rotationalRate);
    return drivetrain
        .applyRequest(
            () -> {
              odometryHealth.setRequestedSpeeds(velocityX, velocityY, rotationalRate);
              return odometryHealth.shouldHoldPosition() ? brake : request;
            })
        .finallyDo(interrupted -> odometryHealth.setRequestedSpeeds(0.0, 0.0, 0.0))
        .withTimeout(seconds);
  }

  private Command holdBrake(double seconds) {
    return drivetrain
        .applyRequest(
            () -> {
              odometryHealth.setRequestedSpeeds(0.0, 0.0, 0.0);
              return brake;
            })
        .withTimeout(seconds);
  }

  public Command getAutonomousCommand() {
    return autonomousChooser.getSelected();
  }
}
