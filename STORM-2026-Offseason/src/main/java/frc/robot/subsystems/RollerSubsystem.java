package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.ConfigMechanisms;
import frc.robot.util.MotorSignals;

public final class RollerSubsystem extends SubsystemBase {
  private final MotorSignals lower = MotorSignals.mechanism(
      "Roller21", ConfigMechanisms.LOWER_ROLLER_ID, false, ConfigMechanisms.ROLLER_KV, 65);
  private final MotorSignals left = MotorSignals.mechanism(
      "Roller22", ConfigMechanisms.LEFT_ROLLER_ID, false, ConfigMechanisms.ROLLER_KV, 65);
  private final MotorSignals right = MotorSignals.mechanism(
      "Roller23", ConfigMechanisms.RIGHT_ROLLER_ID, true, ConfigMechanisms.ROLLER_KV, 65);

  @Override
  public void periodic() {
    lower.sample();
    left.sample();
    right.sample();
  }

  public boolean readyToFeed() {
    return lower.fresh() && left.fresh() && right.fresh();
  }

  public void feed(double scale) {
    double upper = ConfigMechanisms.FEED_UPPER_RPS * scale;
    boolean leftAccepted = left.velocity(upper);
    boolean rightAccepted = right.velocity(upper);
    lower.voltage(leftAccepted && rightAccepted ? ConfigMechanisms.FEED_LOWER_VOLTS * scale : 0);
  }

  public void eject(double scale) {
    left.velocity(ConfigMechanisms.EJECT_UPPER_RPS * scale);
    right.velocity(ConfigMechanisms.EJECT_UPPER_RPS * scale);
    lower.voltage(ConfigMechanisms.EJECT_LOWER_VOLTS * scale);
  }

  public void stop() {
    lower.voltage(0);
    left.velocity(0);
    right.velocity(0);
  }

  public void publishDiagnostics() {
    lower.publishDiagnostics();
    left.publishDiagnostics();
    right.publishDiagnostics();
  }
}

