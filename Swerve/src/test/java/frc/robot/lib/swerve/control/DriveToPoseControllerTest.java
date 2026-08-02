package frc.robot.lib.swerve.control;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

class DriveToPoseControllerTest {
  @Test
  void requiresPoseAndMotionToBeStable() {
    Pose2d target = new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(90));
    assertTrue(DriveToPoseController.isPreciselyReady(target, new ChassisSpeeds(), target));
    assertFalse(DriveToPoseController.isPreciselyReady(target, new ChassisSpeeds(0.2, 0.0, 0.0), target));
    assertFalse(DriveToPoseController.isPreciselyReady(new Pose2d(2.1, 3.0, target.getRotation()), new ChassisSpeeds(), target));
  }
}
