package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.ConfigAutomaticAim;
import frc.robot.config.ConfigMechanisms;
import frc.robot.config.ConfigMechanisms.Shot;
import frc.robot.config.FieldTargets;

/** Único ponto que autoriza os rolos. Chamado uma vez por ciclo, após ler os mecanismos. */
public final class Superstructure {
  public enum Goal { IDLE, COLLECT, HOLD, SHOOT_HUB, OUTTAKE }
  public enum Phase { IDLE, COLLECTING, HOLDING, NAVIGATING, AIMING, LOCKING_X, RECOVERING_RPM,
    FEEDING, EJECTING, CALIBRATION }
  public enum Gate { IDLE, NAVIGATION, ALLIANCE_UNKNOWN, POSE_UNREFERENCED, OUTSIDE_ZONE,
    AIM, MOVING, WHEEL_LOCK, SHOOTER_FEEDBACK, ROLLER_FEEDBACK, POWER_LIMIT, RPM_OR_FLAP, READY }

  private final IntakeSubsystem intake;
  private final RollerSubsystem rollers;
  private final ShooterSubsystem shooter;
  private final MatchAnalyser match = new MatchAnalyser();
  private Goal goal = Goal.IDLE;
  private Phase phase = Phase.IDLE;
  private Gate gate = Gate.IDLE;
  private Rotation2d ejectHeading = Rotation2d.kZero;
  private double goalStartedAt;
  private double readySince = Double.NaN;
  private boolean xRequested;
  private boolean xConfirmed;
  private boolean calibration;
  private double rpmTrim;
  private double flapTrim;
  private double intakeScale = 1;
  private double rollerScale = 1;
  private boolean feed;
  private double intakeTarget;
  private double shotRpm;

  public Superstructure(IntakeSubsystem intake, RollerSubsystem rollers, ShooterSubsystem shooter) {
    this.intake = intake;
    this.rollers = rollers;
    this.shooter = shooter;
  }

  public void request(Goal requested, Rotation2d heading) {
    if (goal == requested) {
      return;
    }
    goal = requested;
    ejectHeading = heading;
    goalStartedAt = Timer.getFPGATimestamp();
    readySince = Double.NaN;
    xRequested = false;
  }

  public Goal goal() { return goal; }
  public Phase phase() { return phase; }
  public Gate gate() { return gate; }
  public MatchAnalyser.Fuel fuel() { return match.fuel(); }
  public boolean wantsX() { return xRequested; }
  public boolean calibration() { return calibration; }
  public double rpmTrim() { return rpmTrim; }
  public double flapTrim() { return flapTrim; }
  public double intakeScale() { return intakeScale; }
  public double rollerScale() { return rollerScale; }

  public void setCalibration(boolean enabled) {
    calibration = enabled;
  }

  public void trim(String parameter, int direction) {
    if (!calibration || Math.abs(direction) != 1) {
      return;
    }
    switch (parameter) {
      case "rpm" -> rpmTrim = MathUtil.clamp(rpmTrim + 50 * direction, -600, 600);
      case "flap" -> flapTrim = MathUtil.clamp(flapTrim + direction, -8, 8);
      case "intake" -> intakeScale = MathUtil.clamp(intakeScale + 0.05 * direction, 0.5, 1.15);
      case "rollers" -> rollerScale = MathUtil.clamp(rollerScale + 0.05 * direction, 0.5, 1.2);
      default -> { }
    }
  }

  public Rotation2d headingForHub(Pose2d pose, Alliance alliance) {
    return FieldTargets.hub(alliance).minus(pose.getTranslation()).getAngle()
        .minus(Rotation2d.fromRadians(ConfigAutomaticAim.SHOOTER_YAW_OFFSET_RADIANS));
  }

  public double headingError(Pose2d pose, Alliance alliance) {
    return headingForHub(pose, alliance).minus(pose.getRotation()).getRadians();
  }

  public double ejectOmega(Pose2d pose, double now) {
    double elapsed = now - goalStartedAt;
    double offset = elapsed < 0.9 ? Math.toRadians(5) * Math.sin(elapsed * 4 * Math.PI) : 0;
    double error = ejectHeading.plus(Rotation2d.fromRadians(offset))
        .minus(pose.getRotation()).getRadians();
    return MathUtil.clamp(5 * error, -0.75, 0.75);
  }

  public void update(Pose2d pose, ChassisSpeeds speeds, Alliance alliance, boolean poseReferenced,
      boolean navigating, boolean brakeApplied, boolean reducedRpm, boolean calibrationSpin) {
    double now = Timer.getFPGATimestamp();
    xConfirmed = brakeApplied;
    feed = false;
    intakeTarget = 0;
    shotRpm = 0;
    phase = Phase.IDLE;
    gate = Gate.IDLE;
    if (DriverStation.isDisabled()) {
      xRequested = false;
      readySince = Double.NaN;
      if (calibration) {
        phase = Phase.CALIBRATION;
      }
      shooter.stop();
    } else if (calibration) {
      phase = Phase.CALIBRATION;
      xRequested = true;
      if (calibrationSpin && brakeApplied && stationary(speeds)) {
        double distance = alliance != null && poseReferenced
            ? pose.getTranslation().getDistance(FieldTargets.hub(alliance)) : 3;
        if (shooter.feedbackReady()) {
          shooter.prepare(ConfigMechanisms.shotAt(distance, rpmTrim, flapTrim));
        } else {
          shooter.stop();
        }
      } else {
        shooter.stop();
      }
    } else {
      switch (goal) {
        case IDLE -> shooter.stop();
        case HOLD -> {
          phase = Phase.HOLDING;
          shooter.stop();
        }
        case COLLECT -> {
          phase = Phase.COLLECTING;
          intakeTarget = Math.min(ConfigMechanisms.COLLECT_RPS * intakeScale,
              ConfigMechanisms.MAX_COLLECT_RPS);
          shooter.stop();
        }
        case OUTTAKE -> {
          phase = Phase.EJECTING;
          intakeTarget = ConfigMechanisms.EJECT_INTAKE_RPS * intakeScale;
          shooter.stop();
          if (now - goalStartedAt > 0.65) {
            match.ejected();
          }
        }
        case SHOOT_HUB -> updateHub(now, pose, speeds, alliance, poseReferenced,
            navigating, brakeApplied, reducedRpm);
      }
    }
    if (goal != Goal.SHOOT_HUB && !calibration) {
      xRequested = false;
      readySince = Double.NaN;
    }
    intake.runAt(intakeTarget);
    if (phase == Phase.EJECTING) {
      rollers.eject(rollerScale);
    } else if (feed) {
      rollers.feed(rollerScale);
    } else {
      rollers.stop();
    }
    match.update(now, phase == Phase.COLLECTING, intakeTarget, intake.speedRps(), intake.currentAmps(),
        feed, shooter.averageRpm(), shotRpm, shooter.currentAmps());
  }

  private void updateHub(double now, Pose2d pose, ChassisSpeeds speeds, Alliance alliance,
      boolean poseReferenced, boolean navigating, boolean brakeApplied, boolean reducedRpm) {
    phase = navigating ? Phase.NAVIGATING : Phase.AIMING;
    if (alliance == null || !poseReferenced || !FieldTargets.insideScoringZone(pose, alliance)
        || navigating) {
      gate = alliance == null ? Gate.ALLIANCE_UNKNOWN : !poseReferenced ? Gate.POSE_UNREFERENCED
          : navigating ? Gate.NAVIGATION : Gate.OUTSIDE_ZONE;
      xRequested = false;
      readySince = Double.NaN;
      shooter.stop();
      return;
    }

    double distance = pose.getTranslation().getDistance(FieldTargets.hub(alliance));
    Shot shot = ConfigMechanisms.shotAt(distance,
        rpmTrim - (reducedRpm ? ConfigMechanisms.REDUCED_SHOT_RPM : 0), flapTrim);
    shotRpm = shot.rpm();
    if (shooter.feedbackReady()) {
      shooter.prepare(shot);
    } else {
      shooter.stop();
    }
    boolean aligned = Math.abs(headingError(pose, alliance)) < Math.toRadians(xRequested ? 3 : 2);
    boolean stopped = stationary(speeds);
    xRequested = aligned && stopped;
    boolean powerAvailable = RobotController.getBatteryVoltage() >= 7.0 && !RobotController.isBrownedOut();
    gate = !aligned ? Gate.AIM : !stopped ? Gate.MOVING
        : !brakeApplied ? Gate.WHEEL_LOCK : !shooter.feedbackReady() ? Gate.SHOOTER_FEEDBACK
        : !rollers.readyToFeed() ? Gate.ROLLER_FEEDBACK : !powerAvailable ? Gate.POWER_LIMIT
        : !shooter.ready(shot) ? Gate.RPM_OR_FLAP : Gate.READY;
    if (gate == Gate.READY) {
      if (!Double.isFinite(readySince)) {
        readySince = now;
      }
      feed = now - readySince >= 0.10;
    } else {
      readySince = Double.NaN;
    }
    if (xRequested) {
      phase = feed ? Phase.FEEDING : !brakeApplied ? Phase.LOCKING_X : Phase.RECOVERING_RPM;
    }
  }

  private static boolean stationary(ChassisSpeeds speeds) {
    return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) < 0.10
        && Math.abs(speeds.omegaRadiansPerSecond) < Math.toRadians(6);
  }

  public void publish() {
    SmartDashboard.putString("Storm/Goal", goal.name());
    SmartDashboard.putString("Storm/Phase", phase.name());
    SmartDashboard.putString("Storm/Gate", gate.name());
    SmartDashboard.putBoolean("Storm/Feeding", feed);
    SmartDashboard.putBoolean("Storm/XLock", xConfirmed);
    SmartDashboard.putNumber("Intake/TargetRPS", intakeTarget);
    SmartDashboard.putNumber("Intake/MeasuredRPS", intake.speedRps());
    match.publish();
  }
}

