package frc.robot.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.junit.jupiter.api.Test;

class HeadingVisionAssistTest {
  @Test
  void interpolatesAtCaptureAndTakesTheShortWayAcross180() {
    var assist = running();
    gyro(assist, 1.02, 178, 15);
    gyro(assist, 1.06, -178, 15);
    assertTrue(assist.beginEvaluation(1.08, 15));
    assist.consider(frame("front", 1.04, -174, 2), Rotation2d.fromDegrees(2), 1.08);
    assertEquals(0.60, assist.finishEvaluation(1.08), 1e-6);
    assertEquals(HeadingVisionAssist.Status.TRACKING, assist.status(1.09));
    assertEquals(HeadingVisionAssist.Status.HOLDING, assist.status(1.70));
  }

  @Test
  void retainsOffsetWhenSwitchedOffAndClearsItOnReferenceReset() {
    var assist = running();
    assertEquals(-0.60, evaluate(assist, 1.10, -10), 1e-6);
    assist.setEnabled(false, 1.11);
    assertFalse(assist.beginEvaluation(1.32, 0));
    assertEquals(-0.60, assist.offsetDegrees(), 1e-6);
    assist.setEnabled(true, 1.33);
    gyro(assist, 1.34, 0, 0);
    gyro(assist, 1.38, 0, 0);
    assertTrue(assist.beginEvaluation(1.39, 0));
    assist.consider(frame("front", 1.32, 10, 2), Rotation2d.kZero, 1.39);
    assertEquals(0, assist.finishEvaluation(1.39));
    assist.resetReference(1.40);
    assertEquals(0, assist.offsetDegrees());
    assertEquals(HeadingVisionAssist.Status.WAITING, assist.status(1.40));
    assist.setTeleop(false, 1.50);
    assertFalse(assist.beginEvaluation(1.70, 0));
  }

  @Test
  void rejectsMissingHistoryFastMotionOneTagAndCameraDisagreement() {
    var assist = running();
    gyro(assist, 1.03, 0, 80);
    gyro(assist, 1.06, 0, 5);
    assertFalse(assist.beginEvaluation(1.08, 80));
    assertTrue(assist.beginEvaluation(1.09, 0));
    assist.consider(frame("front", 1.04, 5, 2), Rotation2d.kZero, 1.09);
    assist.consider(frame("front", 1.08, 5, 2), Rotation2d.kZero, 1.09);
    assist.consider(frame("front", 1.06, 5, 1), Rotation2d.kZero, 1.09);
    assertEquals(0, assist.finishEvaluation(1.09));
    gyro(assist, 1.26, 0, 0);
    gyro(assist, 1.30, 0, 0);
    assertTrue(assist.beginEvaluation(1.31, 0));
    assist.consider(frame("front", 1.27, 8, 2), Rotation2d.kZero, 1.31);
    assist.consider(frame("rear", 1.28, -8, 2), Rotation2d.kZero, 1.31);
    assertEquals(0, assist.finishEvaluation(1.31));
  }

  @Test
  void rejectsFramesCapturedBeforeTheNewReference() {
    var assist = running();
    assertEquals(0.6, evaluate(assist, 1.10, 10), 1e-6);
    assist.resetReference(1.12);
    gyro(assist, 1.13, 0, 0);
    gyro(assist, 1.15, 0, 0);
    assertTrue(assist.beginEvaluation(1.17, 0));
    assist.consider(frame("front", 1.10, 10, 2), Rotation2d.kZero, 1.17);
    assertEquals(0, assist.finishEvaluation(1.17));
  }

  private static HeadingVisionAssist running() {
    var assist = new HeadingVisionAssist();
    assist.setTeleop(true, 1);
    assist.setEnabled(true, 1);
    return assist;
  }

  private static double evaluate(HeadingVisionAssist assist, double now, double visualYaw) {
    gyro(assist, now - 0.06, 0, 0);
    gyro(assist, now - 0.02, 0, 0);
    assertTrue(assist.beginEvaluation(now, 0));
    assist.consider(frame("front", now - 0.04, visualYaw, 2), Rotation2d.kZero, now);
    return assist.finishEvaluation(now);
  }

  private static void gyro(HeadingVisionAssist assist, double time, double angle, double rate) {
    assist.recordGyro(time, Rotation2d.fromDegrees(angle), rate);
  }

  private static VisionObservation frame(String camera, double time, double angle, int tags) {
    return new VisionObservation(camera,
        new Pose3d(2, 2, 0, new Rotation3d(0, 0, Math.toRadians(angle))), time,
        tags == 1 ? new int[] {3} : new int[] {3, 4}, tags, 2, 0.3, 0.05, 0.1, 0.1);
  }
}
