package frc.robot.lib.swerve.control;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.lib.swerve.config.ConfigSwerve;

/** Stateful teleop setpoint filter using PathPlanner's module-aware feasibility solver. */
public final class SwerveSetpointController {
  private final SwerveSetpointGenerator generator;
  private SwerveSetpoint previous;

  public SwerveSetpointController(
      RobotConfig robotConfig, SwerveDriveKinematics kinematics, ChassisSpeeds initialSpeeds) {
    generator =
        robotConfig == null
            ? null
            : new SwerveSetpointGenerator(
                robotConfig,
                RotationsPerSecond.of(ConfigSwerve.MAX_STEER_RATE_ROTATIONS_PER_SECOND));
    previous =
        new SwerveSetpoint(
            initialSpeeds,
            kinematics.toSwerveModuleStates(initialSpeeds),
            DriveFeedforwards.zeros(4));
  }

  public boolean isEnabled() {
    return generator != null;
  }

  public SwerveSetpoint calculate(ChassisSpeeds desiredRobotRelative) {
    if (generator == null) {
      previous =
          new SwerveSetpoint(
              desiredRobotRelative, previous.moduleStates(), DriveFeedforwards.zeros(4));
      return previous;
    }
    previous =
        generator.generateSetpoint(
            previous,
            desiredRobotRelative,
            Seconds.of(0.020),
            Volts.of(Math.max(6.0, RobotController.getBatteryVoltage())));
    return previous;
  }

  public void reset(ChassisSpeeds measuredSpeeds, SwerveDriveKinematics kinematics) {
    previous =
        new SwerveSetpoint(
            measuredSpeeds,
            kinematics.toSwerveModuleStates(measuredSpeeds),
            DriveFeedforwards.zeros(4));
  }
}
