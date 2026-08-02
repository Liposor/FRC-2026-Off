package frc.robot.lib.swerve.control;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.lib.swerve.config.ConfigLocalization;

/** Profiled X/Y/theta controller with an explicit precision-readiness check. */
public final class DriveToPoseController {
  private final ProfiledPIDController xController =
      translationController();
  private final ProfiledPIDController yController =
      translationController();
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          ConfigLocalization.PATH_ROTATION_KP,
          0.0,
          0.0,
          new TrapezoidProfile.Constraints(
              ConfigLocalization.ALIGN_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
              ConfigLocalization.ALIGN_MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED));

  public DriveToPoseController() {
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  private static ProfiledPIDController translationController() {
    return new ProfiledPIDController(
        ConfigLocalization.PATH_TRANSLATION_KP,
        0.0,
        0.0,
        new TrapezoidProfile.Constraints(
            ConfigLocalization.ALIGN_MAX_SPEED_METERS_PER_SECOND,
            ConfigLocalization.ALIGN_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED));
  }

  public void reset(Pose2d currentPose, ChassisSpeeds measuredRobotRelative) {
    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(measuredRobotRelative, currentPose.getRotation());
    xController.reset(currentPose.getX(), fieldSpeeds.vxMetersPerSecond);
    yController.reset(currentPose.getY(), fieldSpeeds.vyMetersPerSecond);
    thetaController.reset(
        currentPose.getRotation().getRadians(), fieldSpeeds.omegaRadiansPerSecond);
  }

  public ChassisSpeeds calculate(Pose2d currentPose, Pose2d targetPose) {
    double vx = xController.calculate(currentPose.getX(), targetPose.getX());
    double vy = yController.calculate(currentPose.getY(), targetPose.getY());
    double omega =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    return ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, currentPose.getRotation());
  }

  public static boolean isPreciselyReady(
      Pose2d currentPose, ChassisSpeeds measuredRobotRelative, Pose2d targetPose) {
    return currentPose.getTranslation().getDistance(targetPose.getTranslation())
            <= ConfigLocalization.ALIGN_POSITION_TOLERANCE_METERS
        && Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians())
            <= ConfigLocalization.ALIGN_HEADING_TOLERANCE_RADIANS
        && Math.hypot(
                measuredRobotRelative.vxMetersPerSecond,
                measuredRobotRelative.vyMetersPerSecond)
            <= ConfigLocalization.ALIGN_MAX_READY_SPEED_METERS_PER_SECOND
        && Math.abs(measuredRobotRelative.omegaRadiansPerSecond)
            <= ConfigLocalization.ALIGN_MAX_READY_OMEGA_RADIANS_PER_SECOND;
  }
}
