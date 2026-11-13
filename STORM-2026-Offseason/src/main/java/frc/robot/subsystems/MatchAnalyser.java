package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.ConfigMechanisms;

/** Indícios de carga mecânica. Contadores representam eventos e não bolas individuais. */
public final class MatchAnalyser {
  public enum Fuel { UNKNOWN, LIKELY_PRESENT, LIKELY_EMPTY }

  private Fuel fuel = Fuel.UNKNOWN;
  private int pickups;
  private int launches;
  private boolean intakeLoaded;
  private boolean shooterLoaded;
  private double collectStartedAt;
  private double lastPickupAt = Double.NaN;
  private double lastLaunchAt = Double.NaN;
  private double feedStartedAt;
  private double lastLoadAt;
  private double lastFeedAt = Double.NEGATIVE_INFINITY;
  private boolean wasCollecting;
  private boolean wasFeeding;
  private double pickupCandidateAt = Double.NaN;

  public void update(double now, boolean collecting, double intakeTargetRps, double intakeRps,
      double intakeAmps, boolean feeding, double rpm, double targetRpm, double shooterAmps) {
    if (collecting && !wasCollecting) {
      collectStartedAt = now;
      intakeLoaded = false;
      pickupCandidateAt = Double.NaN;
    }
    wasCollecting = collecting;
    if (collecting && now - collectStartedAt > 0.3 && Double.isFinite(intakeRps)
        && Double.isFinite(intakeAmps)) {
      boolean contact = intakeAmps > 25 && intakeRps > 8 && intakeRps < intakeTargetRps * 0.93;
      if (contact && !Double.isFinite(pickupCandidateAt)) {
        pickupCandidateAt = now;
      }
      // Uma queda curta seguida de recuperação distingue melhor entrada de bola de travamento.
      if (!contact && Double.isFinite(pickupCandidateAt)) {
        double duration = now - pickupCandidateAt;
        if (!intakeLoaded && duration >= 0.02 && duration < 0.40
            && intakeRps >= intakeTargetRps * 0.85) {
          fuel = Fuel.LIKELY_PRESENT;
          pickups++;
          lastPickupAt = now;
          intakeLoaded = true;
        }
        pickupCandidateAt = Double.NaN;
      }
      if (intakeAmps < 14) {
        intakeLoaded = false;
      }
    }

    if (feeding && !wasFeeding && now - lastLoadAt > 0.15) {
      feedStartedAt = now;
      lastLoadAt = now;
    }
    wasFeeding = feeding;
    if (feeding) {
      lastFeedAt = now;
    }
    // A roda pode acusar a carga um ciclo depois do corte da alimentação.
    boolean recentFeed = feeding || now - lastFeedAt < 0.08;
    if (recentFeed && Double.isFinite(rpm) && Double.isFinite(shooterAmps) && targetRpm > 0) {
      double dip = targetRpm - rpm;
      if (dip > ConfigMechanisms.RPM_READY_TOLERANCE && shooterAmps > 30) {
        if (!shooterLoaded) {
          launches++;
          lastLaunchAt = now;
          fuel = Fuel.LIKELY_PRESENT;
        }
        shooterLoaded = true;
        lastLoadAt = now;
      } else if (dip < 70) {
        shooterLoaded = false;
      }
      if (feeding && now - Math.max(feedStartedAt, lastLoadAt) > 0.9
          && dip < 70 && shooterAmps < 28) {
        fuel = Fuel.LIKELY_EMPTY;
      }
    }
  }

  public void ejected() {
    fuel = Fuel.UNKNOWN;
  }

  public Fuel fuel() {
    return fuel;
  }

  public void publish() {
    SmartDashboard.putString("Match/Fuel", fuel.name());
    SmartDashboard.putNumber("Match/PickupEvents", pickups);
    SmartDashboard.putNumber("Match/LaunchEvents", launches);
    SmartDashboard.putNumber("Match/LastPickupSec", lastPickupAt);
    SmartDashboard.putNumber("Match/LastLaunchSec", lastLaunchAt);
  }
}

