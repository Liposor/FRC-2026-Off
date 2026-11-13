package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.ConfigMechanisms;
import frc.robot.util.MotorSignals;

public final class IntakeSubsystem extends SubsystemBase {
  private final MotorSignals intake =
      MotorSignals.mechanism("Intake32", ConfigMechanisms.INTAKE_ID, false, 0.12, 80);
  private final DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(0);

  @Override
  public void periodic() {
    intake.sample();
  }

  public void runAt(double rps) {
    intake.velocity(rps);
  }

  public double currentAmps() {
    return intake.amps();
  }

  public double speedRps() {
    return intake.fresh() ? intake.rps() : Double.NaN;
  }

  public void publishDiagnostics() {
    intake.publishDiagnostics();
    SmartDashboard.putNumber("Intake/PivotAbsolute",
        pivotEncoder.isConnected() ? pivotEncoder.get() : Double.NaN);
  }
}

