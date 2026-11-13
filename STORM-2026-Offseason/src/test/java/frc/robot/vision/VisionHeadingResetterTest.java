package frc.robot.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.config.ConfigVision;
import frc.robot.vision.VisionHeadingResetter.Source;
import org.junit.jupiter.api.Test;

class VisionHeadingResetterTest {
  @Test
  void mt2ModeRequiresFreshMatchingMt2EvenWhenMt1HasMultipleTags() {
    var mt1 = frame(20, 4.95, 3);
    var staleMt2 = frame(80, 4.5, 3);
    var samples = List.of(new VisionHeadingResetSample("limelight-front", mt1, staleMt2));
    assertTrue(VisionHeadingResetter.evaluate(samples, 5, true, ConfigVision.FIELD_LAYOUT).accepted());
    assertFalse(VisionHeadingResetter.evaluate(
        samples, 5, true, ConfigVision.FIELD_LAYOUT, true).accepted());
  }

  @Test
  void headingComesFromMt1WhenMt2ValidatesTranslation() {
    var sample = new VisionHeadingResetSample("limelight-front", frame(20, 4.95, 3),
        frame(80, 4.96, 3));
    var result = VisionHeadingResetter.evaluate(
        List.of(sample), 5, true, ConfigVision.FIELD_LAYOUT, true);
    assertTrue(result.accepted());
    assertEquals(20, result.heading().getDegrees(), 1e-6);
    assertEquals(Source.MEGATAG1_MULTI_TAG_VALIDATED_BY_MEGATAG2, result.source());
  }

  private static VisionObservation frame(double heading, double timestamp, int tag) {
    return new VisionObservation("limelight-front",
        new Pose3d(2, 2, 0, new Rotation3d(0, 0, Math.toRadians(heading))),
        timestamp, new int[] {tag, 4}, 2, 2, 0.3, 0.05, 0.1, 0.1);
  }
}

