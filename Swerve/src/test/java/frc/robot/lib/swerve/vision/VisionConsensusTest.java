package frc.robot.lib.swerve.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;

class VisionConsensusTest {
  @Test
  void agreesWhenTwoCamerasAreClose() {
    var result =
        VisionConsensus.solve(
            List.of(
                candidate("front", 2.0, 3.0, 10.00),
                candidate("left", 2.1, 3.0, 10.02)));
    assertTrue(result.accepted());
    assertEquals(2, result.supportingCameras());
    assertEquals(2.05, result.pose().getX(), 1e-9);
  }

  @Test
  void rejectsSimultaneousCameraDisagreement() {
    var result =
        VisionConsensus.solve(
            List.of(
                candidate("front", 2.0, 3.0, 10.00),
                candidate("right", 4.0, 3.0, 10.01)));
    assertFalse(result.accepted());
    assertEquals("CAMERA_DISAGREEMENT", result.reason());
  }

  @Test
  void singleCameraIsAcceptedWithInflatedUncertainty() {
    var result = VisionConsensus.solve(List.of(candidate("front", 2.0, 3.0, 10.00)));
    assertTrue(result.accepted());
    assertTrue(result.xyStdDevMeters() > 0.20);
  }

  private static VisionConsensus.Candidate candidate(
      String camera, double x, double y, double timestamp) {
    return new VisionConsensus.Candidate(camera, new Pose2d(x, y, Pose2d.kZero.getRotation()), timestamp, 0.20, 0.8);
  }
}
