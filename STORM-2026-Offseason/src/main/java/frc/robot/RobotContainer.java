package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.config.ConfigSwerve;
import frc.robot.config.FieldTargets;
import frc.robot.control.Navigation;
import frc.robot.control.RebuiltMatchClock;
import frc.robot.control.TouchServer;
import frc.robot.generated.TunerConstants;
import frc.robot.localization.OdometryHealthMonitor;
import frc.robot.localization.OdometryHealthMonitor.HealthState;
import frc.robot.localization.SwerveGeometryCalibrationCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MatchAnalyser.Fuel;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Goal;
import frc.robot.util.MotorSignals;
import frc.robot.util.PerformanceMonitor;
import frc.robot.vision.LimelightSimulation;
import frc.robot.vision.VisionSubsystem;

public final class RobotContainer {
  private final CommandXboxController pilot = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);
  private final CommandSwerveDrivetrain drive = TunerConstants.createDrivetrain();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final RollerSubsystem rollers = new RollerSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final Superstructure superstructure = new Superstructure(intake, rollers, shooter);
  private final OdometryHealthMonitor odometry = new OdometryHealthMonitor(drive);
  private final VisionSubsystem vision = new VisionSubsystem(drive);
  private final Navigation navigation = new Navigation(drive);
  private final SwerveGeometryCalibrationCommand geometryCalibration =
      new SwerveGeometryCalibrationCommand(drive, odometry::setRequestedSpeeds);
  private final Field2d field = new Field2d();
  private final RebuiltMatchClock matchClock = new RebuiltMatchClock();
  private final SendableChooser<Goal> chooser = new SendableChooser<>();
  private final SendableChooser<Command> autoChooser;
  private final TouchServer touch =
      new TouchServer(Filesystem.getDeployDirectory().toPath().resolve("touch"));
  private final List<MotorSignals> driveMotors = new ArrayList<>();
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.Idle idle = new SwerveRequest.Idle();
  private final SwerveRequest.FieldCentric fieldDrive =
      new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);
  private final SwerveRequest.RobotCentric robotDrive =
      new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);
  private SwerveDriveState state;
  private final Translation2d[] moduleLocations = drive.getModuleLocations();
  private Alliance alliance;
  private boolean odometryFresh;
  private boolean fieldReferenced;
  private boolean navForShot;
  private boolean triedShotNavigation;
  private boolean defaultDriveOwnsX;
  private boolean calibrationSpin;
  private boolean slowMode;
  private boolean waitForControlRelease;
  private Goal touchGoal;
  private String touchResult = "Pronto";
  private long actionCount;
  private double nextStatusAt;
  private double nextDiagnosticsAt;
  private double nextProfileAt;
  private double maxSuperstructureMs;
  private double maxSchedulerMs;
  private double lastDriveTime;
  private Translation2d smoothed = Translation2d.kZero;
  private double smoothedOmega;
  private int diagnosticIndex;
  private List<Pose2d> activePath = List.of();

  public RobotContainer() {
    new PerformanceMonitor();
    new LimelightSimulation(() -> drive.getState().Pose);
    if (RobotBase.isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
    drive.setRequestedSpeedsObserver(odometry::setRequestedSpeeds);
    drive.setPathMotionBlockedSupplier(() -> !odometryFresh || odometry.shouldHoldPosition());
    Telemetry telemetry = new Telemetry();
    drive.registerTelemetry(telemetry::telemeterize);
    state = drive.getState();
    String[] moduleNames = {"FL", "FR", "BL", "BR"};
    for (int index = 0; index < moduleNames.length; index++) {
      String name = moduleNames[index];
      driveMotors.add(new MotorSignals(name + "Drive", drive.getModule(index).getDriveMotor(), false));
      driveMotors.add(new MotorSignals(name + "Steer", drive.getModule(index).getSteerMotor(), false));
    }

    chooser.setDefaultOption("Parado", Goal.IDLE);
    chooser.addOption("Coletar", Goal.COLLECT);
    chooser.addOption("Segurar", Goal.HOLD);
    chooser.addOption("Atirar no HUB", Goal.SHOOT_HUB);
    chooser.addOption("Ejetar", Goal.OUTTAKE);
    SmartDashboard.putData("Estado solicitado", chooser);
    autoChooser = drive.isPathPlannerConfigured() ? AutoBuilder.buildAutoChooser()
        : new SendableChooser<>();
    autoChooser.setDefaultOption("Parado", Commands.none());
    SmartDashboard.putData("Autonomo", autoChooser);
    SmartDashboard.putData("Campo REBUILT", field);
    SmartDashboard.putBoolean("PathPlanner/Configured", drive.isPathPlannerConfigured());
    PathPlannerLogging.setLogActivePathCallback(poses -> {
      activePath = List.copyOf(poses);
      field.getObject("Trajeto").setPoses(poses);
    });
    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(drive.applyRequest(this::driveRequest).finallyDo(interrupted -> {
      resetSmoothing();
      defaultDriveOwnsX = false;
    }));
    RobotModeTriggers.disabled().whileTrue(drive.applyRequest(() -> idle).ignoringDisable(true));
    pilot.rightStick().and(pilot.start().negate()).onTrue(vision.resetHeadingFromVisionCommand());
    pilot.rightStick().and(pilot.start()).onTrue(vision.resetHeadingWithMegaTag2ValidationCommand());
    pilot.y().onTrue(Commands.runOnce(
        () -> vision.setHeadingAssistEnabled(!vision.isHeadingAssistEnabled())).ignoringDisable(true));
    pilot.leftBumper().onTrue(drive.runOnce(drive::seedFieldCentric));
    pilot.leftStick().onTrue(Commands.runOnce(() -> slowMode = !slowMode));
    pilot.back().onTrue(drive.runOnce(odometry::applyRecommendedRecoveryPose));
    pilot.povRight().and(DriverStation::isTestEnabled).onTrue(geometryCalibration);
    pilot.povLeft().onTrue(Commands.runOnce(geometryCalibration::cancel));
  }

  public void beforeScheduler() {
    state = drive.getState();
    double now = Timer.getFPGATimestamp();
    alliance = DriverStation.getAlliance().orElse(null);
    var clock = matchClock.read();
    double age = Utils.getCurrentTimeSeconds() - state.Timestamp;
    odometryFresh = state.SuccessfulDaqs > 2 && age >= -0.02 && age < 0.12
        && Double.isFinite(state.Pose.getX()) && Double.isFinite(state.Pose.getY())
        && Double.isFinite(state.Pose.getRotation().getRadians());
    fieldReferenced = odometryFresh && vision.hasFieldReference()
        && odometry.getHealthState() != HealthState.BLOCKED;
    if (DriverStation.isFMSAttached() || DriverStation.isAutonomousEnabled()) {
      superstructure.setCalibration(false);
      calibrationSpin = false;
    }
    for (int index = 0; index < 4; index++) {
      TouchServer.Request request = touch.poll();
      if (request == null) {
        break;
      }
      if (request.fresh()) {
        processTouch(request.action());
        actionCount++;
      }
    }
    if (!remoteHeld()) {
      touchGoal = null;
    }
    if (DriverStation.isTeleopEnabled()) {
      updateTeleopGoal();
    } else {
      superstructure.request(Goal.IDLE, state.Pose.getRotation());
    }
    if (navigation.active()
        && (!fieldReferenced || alliance == null || !DriverStation.isTeleopEnabled() || manualOverride()
            || (navForShot && superstructure.goal() != Goal.SHOOT_HUB)
            || (!navForShot && (!remoteHeld() || superstructure.goal() == Goal.SHOOT_HUB
                || superstructure.goal() == Goal.OUTTAKE)))) {
      navigation.cancel();
      navForShot = false;
      if (touchGoal == Goal.COLLECT) {
        touchGoal = null;
        superstructure.request(Goal.IDLE, state.Pose.getRotation());
      }
    }
    if (!navigation.active() && !navForShot && touchGoal == Goal.COLLECT) {
      touchGoal = null;
      superstructure.request(Goal.IDLE, state.Pose.getRotation());
    }
    if (superstructure.goal() != Goal.SHOOT_HUB) {
      triedShotNavigation = false;
    } else if (!navigation.active() && !triedShotNavigation && fieldReferenced && alliance != null
        && !FieldTargets.insideScoringZone(state.Pose, alliance) && !manualOverride()) {
      triedShotNavigation = true;
      navForShot = navigation.nearestShootingPoint(state.Pose, alliance);
    }
    if (now >= nextStatusAt) {
      nextStatusAt = now + 0.1;
      publishStatus(clock);
    }
  }

  public void afterScheduler(double schedulerMs) {
    double startedAt = Timer.getFPGATimestamp();
    boolean spin = calibrationSpin && operator.getHID().getRightBumperButton()
        && (DriverStation.isTestEnabled() || DriverStation.isTeleopEnabled());
    superstructure.update(state.Pose, state.Speeds, alliance, fieldReferenced,
        navigation.active(), xAchieved(), operator.getHID().getXButton(), spin);
    maxSchedulerMs = Math.max(maxSchedulerMs, schedulerMs);
    maxSuperstructureMs = Math.max(maxSuperstructureMs,
        (Timer.getFPGATimestamp() - startedAt) * 1000);
    if (startedAt >= nextProfileAt) {
      nextProfileAt = startedAt + 1;
      SmartDashboard.putNumber("Performance/SchedulerMaxMs", maxSchedulerMs);
      SmartDashboard.putNumber("Performance/SuperstructureMaxMs", maxSuperstructureMs);
      maxSchedulerMs = 0;
      maxSuperstructureMs = 0;
    }
    if (startedAt >= nextDiagnosticsAt) {
      nextDiagnosticsAt = startedAt + 0.10;
      // Distribui leitura/publicação de diagnósticos para evitar rajadas no mesmo ciclo.
      switch (diagnosticIndex) {
        case 0 -> intake.publishDiagnostics();
        case 1 -> rollers.publishDiagnostics();
        case 2 -> shooter.publishDiagnostics();
        default -> driveMotors.get(diagnosticIndex - 3).publishDiagnostics();
      }
      diagnosticIndex = (diagnosticIndex + 1) % (driveMotors.size() + 3);
    }
  }

  private void updateTeleopGoal() {
    Goal wanted = Goal.IDLE;
    if (waitForControlRelease && pilot.getLeftTriggerAxis() <= 0.55
        && pilot.getRightTriggerAxis() <= 0.55 && !operator.getHID().getAButton()
        && !operator.getHID().getBButton()) {
      waitForControlRelease = false;
    }
    if (pilot.getHID().getBButton() || superstructure.calibration() || waitForControlRelease) {
      navigation.cancel();
      touchGoal = null;
    } else if (pilot.getLeftTriggerAxis() > 0.55) {
      wanted = Goal.SHOOT_HUB;
    } else if (operator.getHID().getBButton()) {
      wanted = Goal.OUTTAKE;
    } else if (pilot.getRightTriggerAxis() > 0.55) {
      wanted = Goal.COLLECT;
    } else if (remoteHeld() && touchGoal != null) {
      wanted = touchGoal;
    } else if (operator.getHID().getAButton()) {
      Goal chosen = chooser.getSelected();
      wanted = chosen != null ? chosen : Goal.IDLE;
    } else if (superstructure.fuel() == Fuel.LIKELY_PRESENT) {
      wanted = Goal.HOLD;
    }
    if (manualOverride() && (wanted == Goal.SHOOT_HUB || wanted == Goal.OUTTAKE)) {
      wanted = Goal.IDLE;
      touchGoal = null;
    }
    superstructure.request(wanted, state.Pose.getRotation());
  }

  private boolean remoteHeld() {
    return DriverStation.isJoystickConnected(1) && operator.getHID().getLeftBumperButton();
  }

  private boolean manualOverride() {
    return pilot.getHID().getBButton() || pilot.getHID().getAButton()
        || Math.hypot(pilot.getLeftX(), pilot.getLeftY()) > 0.24
        || Math.abs(pilot.getRightX()) > 0.24;
  }

  private SwerveRequest driveRequest() {
    double now = Timer.getFPGATimestamp();
    double dt = MathUtil.clamp(now - lastDriveTime, 0, 0.04);
    lastDriveTime = now;
    defaultDriveOwnsX = false;
    if (!DriverStation.isTeleopEnabled() || superstructure.calibration()
        || pilot.getHID().getAButton() || (superstructure.wantsX() && !manualOverride())) {
      resetSmoothing();
      defaultDriveOwnsX = true;
      return brake;
    }

    Goal goal = superstructure.goal();
    boolean aim = goal == Goal.SHOOT_HUB && fieldReferenced && alliance != null
        && FieldTargets.insideScoringZone(state.Pose, alliance);
    boolean eject = goal == Goal.OUTTAKE && odometryFresh;
    double max = ConfigSwerve.TELEOP_MAX_SPEED.in(MetersPerSecond);
    double maximumOmega = ConfigSwerve.TELEOP_MAX_ANGULAR_RATE.in(RadiansPerSecond);
    if (goal == Goal.COLLECT) {
      max *= ConfigSwerve.COLLECTING_PROFILE.translationScale();
      maximumOmega *= ConfigSwerve.COLLECTING_PROFILE.rotationScale();
    }
    if (slowMode || !odometryFresh) {
      max *= 0.4;
      maximumOmega *= 0.4;
    }

    Translation2d desired = new Translation2d(-pilot.getLeftY(), -pilot.getLeftX());
    double magnitude = MathUtil.applyDeadband(Math.min(1, desired.getNorm()), ConfigSwerve.JOYSTICK_DEADBAND);
    desired = magnitude > 0 ? new Translation2d(magnitude * max, desired.getAngle()) : Translation2d.kZero;
    if (aim || eject) {
      smoothed = Translation2d.kZero;
    } else {
      Translation2d delta = desired.minus(smoothed);
      double step = (desired.getNorm() < smoothed.getNorm() ? 7.0 : 4.5) * dt;
      smoothed = delta.getNorm() > step
          ? smoothed.plus(new Translation2d(step, delta.getAngle())) : desired;
      if (smoothed.getNorm() > max) {
        smoothed = new Translation2d(max, smoothed.getAngle());
      }
    }
    double omega = -MathUtil.applyDeadband(pilot.getRightX(), ConfigSwerve.JOYSTICK_DEADBAND)
        * maximumOmega;
    if (aim) {
      omega = MathUtil.clamp(5 * superstructure.headingError(state.Pose, alliance)
          - 0.12 * state.Speeds.omegaRadiansPerSecond, -1.8, 1.8);
    } else if (eject) {
      omega = superstructure.ejectOmega(state.Pose, now);
    }
    smoothedOmega = MathUtil.clamp(smoothedOmega
        + MathUtil.clamp(omega - smoothedOmega, -12 * dt, 12 * dt), -maximumOmega, maximumOmega);
    odometry.setRequestedSpeeds(smoothed.getX(), smoothed.getY(), smoothedOmega);
    return odometryFresh
        ? fieldDrive.withVelocityX(smoothed.getX()).withVelocityY(smoothed.getY()).withRotationalRate(smoothedOmega)
        : robotDrive.withVelocityX(smoothed.getX()).withVelocityY(smoothed.getY()).withRotationalRate(smoothedOmega);
  }

  private boolean xAchieved() {
    if (!defaultDriveOwnsX || !odometryFresh || state.ModuleStates.length != 4) {
      return false;
    }
    for (int index = 0; index < 4; index++) {
      Rotation2d expected = moduleLocations[index].getAngle();
      double angle = state.ModuleStates[index].angle.minus(expected).getRadians();
      if (Math.abs(MathUtil.inputModulus(angle, -Math.PI / 2, Math.PI / 2)) > Math.toRadians(10)
          || Math.abs(state.ModuleStates[index].speedMetersPerSecond) > 0.12) {
        return false;
      }
    }
    return true;
  }

  private void resetSmoothing() {
    smoothed = Translation2d.kZero;
    smoothedOmega = 0;
    lastDriveTime = Timer.getFPGATimestamp();
    odometry.setRequestedSpeeds(0, 0, 0);
  }

  private void processTouch(String action) {
    touchResult = "Pedido recusado";
    if (action.equals("stop")) {
      navigation.cancel();
      touchGoal = null;
      calibrationSpin = false;
      waitForControlRelease = true;
      touchResult = "Pedido da tela cancelado";
    } else if (action.equals("heading-on") || action.equals("heading-off")) {
      boolean enabled = action.equals("heading-on");
      vision.setHeadingAssistEnabled(enabled);
      touchResult = enabled ? "Correção de heading ligada" : "Correção de heading desligada";
    } else if (action.equals("calibration-off")) {
      superstructure.setCalibration(false);
      calibrationSpin = false;
      touchResult = "Calibração encerrada";
    } else if (action.equals("calibration-on") && !DriverStation.isFMSAttached()
        && (DriverStation.isDisabled() || DriverStation.isTestEnabled())) {
      superstructure.setCalibration(true);
      navigation.cancel();
      touchGoal = null;
      touchResult = "Calibração aberta";
    } else if (action.equals("flap-zero") && superstructure.calibration() && DriverStation.isDisabled()) {
      touchResult = shooter.referenceAtMechanicalZero() ? "Zero do flap gravado para esta sessão"
          : "Falha na referência do flap";
    } else if (action.startsWith("trim:") && superstructure.calibration()) {
      String[] fields = action.split(":");
      if (fields.length == 3 && (fields[2].equals("1") || fields[2].equals("-1"))) {
        superstructure.trim(fields[1], Integer.parseInt(fields[2]));
        touchResult = "Ajuste aplicado";
      }
    } else if (action.equals("spin") && superstructure.calibration()) {
      calibrationSpin = !calibrationSpin;
      touchResult = calibrationSpin ? "Segure RB no controle 2" : "Bancada desarmada";
    } else if (remoteHeld() && DriverStation.isTeleopEnabled() && !superstructure.calibration()
        && !manualOverride()) {
      processRemoteMotion(action);
    } else {
      touchResult = "Habilite o teleop e segure LB no controle 2";
    }
  }

  private void processRemoteMotion(String action) {
    if (action.startsWith("state:")) {
      try {
        Goal wanted = Goal.valueOf(action.substring(6));
        navigation.cancel();
        navForShot = false;
        touchGoal = wanted;
        touchResult = "Estado solicitado";
      } catch (IllegalArgumentException error) {
        touchResult = "Estado inválido";
      }
      return;
    }
    if (!fieldReferenced || alliance == null) {
      touchResult = "Referencie a pose e confirme a aliança";
      return;
    }
    if (superstructure.goal() == Goal.SHOOT_HUB || superstructure.goal() == Goal.OUTTAKE
        || pilot.getRightTriggerAxis() > 0.55 || pilot.getLeftTriggerAxis() > 0.55) {
      touchResult = "Solte os comandos de mecanismo para iniciar uma rota";
      return;
    }
    boolean started = false;
    navForShot = false;
    touchGoal = null;
    if (action.startsWith("route:")) {
      started = navigation.collect(state.Pose, alliance, action.substring(6));
      if (started) {
        touchGoal = Goal.COLLECT;
      }
    } else if (action.matches("point:[1-4]")) {
      started = navigation.toPoint(state.Pose,
          FieldTargets.shootingPoints(alliance).get(Integer.parseInt(action.substring(6)) - 1));
    } else if (action.startsWith("goto:")) {
      String[] fields = action.split(":");
      try {
        if (fields.length == 3) {
          Pose2d destination = new Pose2d(Double.parseDouble(fields[1]),
              Double.parseDouble(fields[2]), state.Pose.getRotation());
          started = navigation.toPoint(state.Pose, destination);
        }
      } catch (NumberFormatException error) {
        touchResult = "Destino inválido";
        return;
      }
    }
    touchResult = started ? "Rota iniciada" : "Rota recusada: " + navigation.result().name();
  }

  @SuppressWarnings("unchecked")
  private void publishStatus(RebuiltMatchClock.Reading clock) {
    superstructure.publish();
    field.setRobotPose(state.Pose);
    field.getObject("Pontos de tiro").setPoses(
        alliance == null ? List.of() : FieldTargets.shootingPoints(alliance));
    Pose2d destination = navigation.target();
    field.getObject("Destino").setPoses(destination == null ? List.of() : List.of(destination));
    if (destination == null && DriverStation.isTeleop()) {
      field.getObject("Trajeto").setPoses(List.of());
      activePath = List.of();
    }

    double battery = RobotController.getBatteryVoltage();
    String allianceName = alliance == null ? "UNKNOWN" : alliance.name();
    SmartDashboard.putNumber("Storm/BatteryV", battery);
    SmartDashboard.putString("Storm/Alliance", allianceName);
    SmartDashboard.putString("Storm/Navigation", navigation.result().name());
    SmartDashboard.putBoolean("Storm/FieldReferenced", fieldReferenced);
    SmartDashboard.putBoolean("Storm/PoseFeedbackFresh", odometryFresh);
    SmartDashboard.putNumber("Storm/RPMTrim", superstructure.rpmTrim());
    SmartDashboard.putNumber("Storm/FlapTrimDeg", superstructure.flapTrim());
    SmartDashboard.putNumber("Match/TimeSec", clock.matchSeconds());
    SmartDashboard.putString("Match/Shift", clock.shift().name());
    SmartDashboard.putNumber("Match/ShiftTimeSec", clock.shiftSeconds());
    SmartDashboard.putString("Match/Hub", clock.hub().name());
    SmartDashboard.putString("Match/ClockSource", clock.source().name());

    JSONObject data = new JSONObject();
    data.put("battery", battery);
    data.put("alliance", allianceName);
    data.put("matchTime", clock.matchSeconds());
    data.put("shift", clock.shift().name());
    data.put("shiftTime", clock.shiftSeconds());
    data.put("hub", clock.hub().name());
    data.put("clockSource", clock.source().name());
    data.put("enabled", DriverStation.isEnabled());
    data.put("teleop", DriverStation.isTeleopEnabled());
    data.put("canCalibrate", !DriverStation.isFMSAttached()
        && (DriverStation.isDisabled() || DriverStation.isTestEnabled()));
    data.put("remote", remoteHeld());
    data.put("fieldReady", fieldReferenced);
    data.put("headingAssist", vision.isHeadingAssistEnabled());
    data.put("headingAssistStatus", vision.headingAssistStatus());
    data.put("headingAssistOffset", vision.headingAssistOffsetDegrees());
    data.put("calibration", superstructure.calibration());
    data.put("spin", calibrationSpin);
    data.put("rpm", superstructure.rpmTrim());
    data.put("flap", superstructure.flapTrim());
    data.put("intake", Math.round(superstructure.intakeScale() * 100));
    data.put("rollers", Math.round(superstructure.rollerScale() * 100));
    data.put("goal", superstructure.goal().name());
    data.put("phase", superstructure.phase().name());
    data.put("gate", superstructure.gate().name());
    data.put("fuel", superstructure.fuel().name());
    data.put("nav", navigation.result().name());
    data.put("pose", poseJson(state.Pose));
    data.put("target", destination == null ? null : poseJson(destination));
    JSONArray path = new JSONArray();
    int stride = Math.max(1, (activePath.size() + 55) / 56);
    for (int index = 0; index < activePath.size(); index += stride) {
      var sample = poseJson(activePath.get(index));
      if (sample != null) {
        path.add(sample);
      }
    }
    data.put("path", path);
    JSONArray points = new JSONArray();
    if (alliance != null) {
      FieldTargets.shootingPoints(alliance).forEach(point -> points.add(poseJson(point)));
    }
    data.put("points", points);
    JSONArray routes = new JSONArray();
    routes.addAll(navigation.collectRoutes());
    data.put("routes", routes);
    data.put("result", touchResult);
    data.put("actionCount", actionCount);
    data.put("token", touch.token());
    touch.publish(data.toJSONString());
  }

  private static List<Double> poseJson(Pose2d pose) {
    double x = pose.getX();
    double y = pose.getY();
    double degrees = pose.getRotation().getDegrees();
    return Double.isFinite(x) && Double.isFinite(y) && Double.isFinite(degrees)
        ? List.of(x, y, degrees) : null;
  }

  public void modeChanged() {
    navigation.cancel();
    touchGoal = null;
    calibrationSpin = false;
    touch.clear();
    triedShotNavigation = false;
    navForShot = false;
    waitForControlRelease = false;
    superstructure.request(Goal.IDLE, state.Pose.getRotation());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

