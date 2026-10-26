package frc.robot.lib.swerve.localization;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class CollisionConfidenceTest {
  @Test
  void risesForImpactAndLargeCommandMismatch() {
    double nominal = CollisionConfidence.calculate(2.7, 2.65, 0.2, 2.0, 0.4);
    double blocked = CollisionConfidence.calculate(2.7, 0.0, 5.0, 40.0, 3.0);
    assertTrue(blocked > nominal);
    assertTrue(blocked > 0.70);
  }
}
