package frc.robot.control;

import static frc.robot.control.RebuiltMatchClock.calculate;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.control.RebuiltMatchClock.Hub;
import frc.robot.control.RebuiltMatchClock.Mode;
import frc.robot.control.RebuiltMatchClock.Shift;
import frc.robot.control.RebuiltMatchClock.Source;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

class RebuiltMatchClockTest {
  @ParameterizedTest
  @CsvSource({
    "140, TRANSITION, 10",
    "130.01, TRANSITION, 0.01",
    "130, SHIFT_1, 25",
    "105, SHIFT_2, 25",
    "80, SHIFT_3, 25",
    "55, SHIFT_4, 25",
    "30, ENDGAME, 30",
    "0, ENDGAME, 0"
  })
  void countdownResetsAtOfficialBoundaries(double remaining, Shift shift, double shiftTime) {
    var clock = calculate(Mode.TELEOP, remaining, 0, true, Alliance.Blue, "R");
    assertEquals(shift, clock.shift());
    assertEquals(remaining, clock.matchSeconds());
    assertEquals(shiftTime, clock.shiftSeconds(), 0.0001);
  }

  @Test
  void hubAlternatesAccordingToTheFirstInactiveAlliance() {
    assertEquals(Hub.INACTIVE, calculate(Mode.TELEOP, 130, 0, true, Alliance.Red, "R").hub());
    assertEquals(Hub.ACTIVE, calculate(Mode.TELEOP, 105, 0, true, Alliance.Red, "R").hub());
    assertEquals(Hub.INACTIVE, calculate(Mode.TELEOP, 80, 0, true, Alliance.Blue, "B").hub());
    assertEquals(Hub.ACTIVE, calculate(Mode.TELEOP, 55, 0, true, Alliance.Blue, "B").hub());
    assertEquals(Hub.ACTIVE, calculate(Mode.TELEOP, 140, 0, true, Alliance.Blue, "").hub());
    assertEquals(Hub.ACTIVE, calculate(Mode.TELEOP, 30, 0, true, Alliance.Blue, "").hub());
    assertEquals(Hub.UNKNOWN, calculate(Mode.TELEOP, 130, 0, true, Alliance.Blue, "").hub());
    assertEquals(Hub.UNKNOWN, calculate(Mode.TELEOP, 130, 0, true, null, "R").hub());
  }

  @Test
  void autoUsesItsOwnCountdownAndPracticeFallbackIsLabeled() {
    var auto = calculate(Mode.AUTO, 17, 0, true, Alliance.Blue, "");
    assertEquals(Shift.AUTO, auto.shift());
    assertEquals(17, auto.shiftSeconds());
    assertEquals(Hub.ACTIVE, auto.hub());
    var practice = calculate(Mode.TELEOP, -1, 134, false, Alliance.Blue, "");
    assertEquals(Source.PRACTICE, practice.source());
    assertEquals(4, practice.shiftSeconds());
  }

  @Test
  void unavailableFmsTimeAndDisabledModesDoNotInventAClock() {
    var missing = calculate(Mode.TELEOP, -1, 134, true, Alliance.Blue, "R");
    assertEquals(Shift.NONE, missing.shift());
    assertEquals(-1, missing.matchSeconds());
    assertEquals(Source.NONE, missing.source());
    assertEquals(Shift.NONE, calculate(Mode.DISABLED, 120, 120, true, Alliance.Blue, "R").shift());
  }
}
