package frc.robot.control;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

/** Tabela 6-2 do REBUILT. Os relógios de AUTO e TELEOP são independentes. */
public final class RebuiltMatchClock {
  static final double AUTO_SECONDS = 20;
  static final double TELEOP_SECONDS = 140;

  enum Mode { DISABLED, AUTO, TELEOP }

  public enum Shift {
    NONE(0), AUTO(0), TRANSITION(130), SHIFT_1(105), SHIFT_2(80),
    SHIFT_3(55), SHIFT_4(30), ENDGAME(0);

    private final double endsAt;

    Shift(double endsAt) {
      this.endsAt = endsAt;
    }
  }

  public enum Hub { UNKNOWN, ACTIVE, INACTIVE }
  public enum Source { NONE, DRIVER_STATION, PRACTICE }

  public record Reading(
      Shift shift, double matchSeconds, double shiftSeconds, Hub hub, Source source) {}

  private Mode lastMode = Mode.DISABLED;
  private double startedAt;

  public Reading read() {
    Mode mode = DriverStation.isAutonomousEnabled() ? Mode.AUTO
        : DriverStation.isTeleopEnabled() ? Mode.TELEOP : Mode.DISABLED;
    double now = Timer.getFPGATimestamp();
    if (mode != lastMode) {
      lastMode = mode;
      startedAt = now;
    }
    double duration = mode == Mode.AUTO ? AUTO_SECONDS : TELEOP_SECONDS;
    return calculate(mode, DriverStation.getMatchTime(), duration - (now - startedAt),
        DriverStation.isFMSAttached(), DriverStation.getAlliance().orElse(null),
        DriverStation.getGameSpecificMessage());
  }

  static Reading calculate(Mode mode, double dsSeconds, double practiceSeconds, boolean fms,
      Alliance alliance, String gameData) {
    if (mode == Mode.DISABLED) {
      return new Reading(Shift.NONE, -1, -1, Hub.UNKNOWN, Source.NONE);
    }
    double duration = mode == Mode.AUTO ? AUTO_SECONDS : TELEOP_SECONDS;
    boolean validTime = Double.isFinite(dsSeconds) && dsSeconds >= 0 && dsSeconds <= duration;
    if (!validTime && fms) {
      return new Reading(mode == Mode.AUTO ? Shift.AUTO : Shift.NONE, -1, -1,
          mode == Mode.AUTO ? Hub.ACTIVE : Hub.UNKNOWN, Source.NONE);
    }
    double remaining = validTime ? dsSeconds : Math.max(0, Math.min(duration, practiceSeconds));
    Shift shift = mode == Mode.AUTO ? Shift.AUTO : teleopShift(remaining);
    return new Reading(shift, remaining, Math.max(0, remaining - shift.endsAt),
        ownHub(shift, alliance, gameData), validTime ? Source.DRIVER_STATION : Source.PRACTICE);
  }

  private static Shift teleopShift(double remaining) {
    if (remaining > Shift.TRANSITION.endsAt) return Shift.TRANSITION;
    if (remaining > Shift.SHIFT_1.endsAt) return Shift.SHIFT_1;
    if (remaining > Shift.SHIFT_2.endsAt) return Shift.SHIFT_2;
    if (remaining > Shift.SHIFT_3.endsAt) return Shift.SHIFT_3;
    if (remaining > Shift.SHIFT_4.endsAt) return Shift.SHIFT_4;
    return Shift.ENDGAME;
  }

  private static Hub ownHub(Shift shift, Alliance alliance, String gameData) {
    if (shift == Shift.AUTO || shift == Shift.TRANSITION || shift == Shift.ENDGAME) {
      return Hub.ACTIVE;
    }
    Alliance inactiveFirst = switch (gameData) {
      case "R" -> Alliance.Red;
      case "B" -> Alliance.Blue;
      default -> null;
    };
    if (alliance == null || inactiveFirst == null) {
      return Hub.UNKNOWN;
    }
    boolean oddShift = shift == Shift.SHIFT_1 || shift == Shift.SHIFT_3;
    return (alliance == inactiveFirst) == oddShift ? Hub.INACTIVE : Hub.ACTIVE;
  }
}
