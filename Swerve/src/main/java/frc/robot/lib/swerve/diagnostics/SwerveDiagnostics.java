package frc.robot.lib.swerve.diagnostics;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.signals.MagnetHealthValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.swerve.config.SwerveHardwareConfig;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/** Low-rate electrical and sensor health telemetry; control signals stay on the CTRE odometry thread. */
public final class SwerveDiagnostics extends SubsystemBase {
  private static final String[] MODULE_NAMES = {"FrontLeft", "FrontRight", "BackLeft", "BackRight"};
  private static final double UPDATE_PERIOD_SECONDS = 0.20;

  private final SwerveSubsystem drivetrain;
  private final ModuleSignals[] modules = new ModuleSignals[4];
  private final PowerDistribution pdh;
  private double lastUpdateSeconds = Double.NEGATIVE_INFINITY;

  public SwerveDiagnostics(SwerveSubsystem drivetrain) {
    this.drivetrain = drivetrain;
    for (int i = 0; i < modules.length; i++) {
      var module = drivetrain.getModule(i);
      modules[i] =
          new ModuleSignals(
              module.getDriveMotor().getSupplyCurrent(),
              module.getDriveMotor().getStatorCurrent(),
              module.getDriveMotor().getMotorVoltage(),
              module.getDriveMotor().getDeviceTemp(),
              module.getSteerMotor().getSupplyCurrent(),
              module.getSteerMotor().getDeviceTemp(),
              module.getEncoder().getAbsolutePosition(),
              module.getEncoder().getMagnetHealth());
      modules[i].setUpdateFrequency(5.0);
    }
    pdh =
        RobotBase.isReal()
            ? new PowerDistribution(
                SwerveHardwareConfig.PDH_CAN_ID, PowerDistribution.ModuleType.kRev)
            : null;
  }

  @Override
  public void periodic() {
    double now = Timer.getFPGATimestamp();
    if (now - lastUpdateSeconds < UPDATE_PERIOD_SECONDS) return;
    lastUpdateSeconds = now;

    var state = drivetrain.getState();
    for (int i = 0; i < modules.length; i++) {
      ModuleSignals signals = modules[i];
      signals.refresh();
      String prefix = "Swerve/Diagnostics/" + MODULE_NAMES[i] + "/";
      SmartDashboard.putNumber(prefix + "DriveSupplyCurrentA", signals.driveSupplyCurrent.getValue().in(Amps));
      SmartDashboard.putNumber(prefix + "DriveStatorCurrentA", signals.driveStatorCurrent.getValue().in(Amps));
      SmartDashboard.putNumber(prefix + "DriveVoltage", signals.driveVoltage.getValue().in(Volts));
      SmartDashboard.putNumber(prefix + "DriveTempC", signals.driveTemperature.getValue().in(Celsius));
      SmartDashboard.putNumber(prefix + "SteerSupplyCurrentA", signals.steerSupplyCurrent.getValue().in(Amps));
      SmartDashboard.putNumber(prefix + "SteerTempC", signals.steerTemperature.getValue().in(Celsius));
      SmartDashboard.putNumber(prefix + "CANcoderAbsoluteRot", signals.absolutePosition.getValue().in(Rotations));
      SmartDashboard.putString(prefix + "MagnetHealth", signals.magnetHealth.getValue().toString());
      double speedError = state.ModuleTargets[i].speedMetersPerSecond - state.ModuleStates[i].speedMetersPerSecond;
      double angleError = MathUtil.angleModulus(state.ModuleTargets[i].angle.minus(state.ModuleStates[i].angle).getRadians());
      SmartDashboard.putNumber(prefix + "SpeedErrorMps", speedError);
      SmartDashboard.putNumber(prefix + "AngleErrorRad", angleError);
    }

    if (pdh != null) {
      SmartDashboard.putNumber("Power/PDH/Voltage", pdh.getVoltage());
      SmartDashboard.putNumber("Power/PDH/TotalCurrentA", pdh.getTotalCurrent());
      SmartDashboard.putNumber("Power/PDH/TemperatureC", pdh.getTemperature());
      SmartDashboard.putBoolean("Power/PDH/SwitchableChannel", pdh.getSwitchableChannel());
    }
  }

  private record ModuleSignals(
      StatusSignal<Current> driveSupplyCurrent,
      StatusSignal<Current> driveStatorCurrent,
      StatusSignal<Voltage> driveVoltage,
      StatusSignal<Temperature> driveTemperature,
      StatusSignal<Current> steerSupplyCurrent,
      StatusSignal<Temperature> steerTemperature,
      StatusSignal<Angle> absolutePosition,
      StatusSignal<MagnetHealthValue> magnetHealth) {
    void setUpdateFrequency(double hz) {
      BaseStatusSignal.setUpdateFrequencyForAll(
          hz,
          driveSupplyCurrent,
          driveStatorCurrent,
          driveVoltage,
          driveTemperature,
          steerSupplyCurrent,
          steerTemperature,
          absolutePosition,
          magnetHealth);
    }

    void refresh() {
      BaseStatusSignal.refreshAll(
          driveSupplyCurrent,
          driveStatorCurrent,
          driveVoltage,
          driveTemperature,
          steerSupplyCurrent,
          steerTemperature,
          absolutePosition,
          magnetHealth);
    }
  }
}
