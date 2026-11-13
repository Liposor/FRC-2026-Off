package frc.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Sinais Phoenix reutilizados. Status do controle e diagnósticos têm taxas independentes. */
public final class MotorSignals {
  private final TalonFX motor;
  private final StatusSignal<?> speed;
  private final StatusSignal<?> current;
  private final StatusSignal<?> temperature;
  private final StatusSignal<Integer> faults;
  private final StatusSignal<Integer> stickyFaults;
  private final BaseStatusSignal[] fastSignals;
  private final BaseStatusSignal[] diagnosticSignals;
  private final VelocityVoltage velocity = new VelocityVoltage(0).withEnableFOC(true);
  private final VoltageOut voltage = new VoltageOut(0);
  private final NeutralOut neutral = new NeutralOut();
  private final String key;
  private boolean configured = true;
  private boolean commandAccepted = true;

  public MotorSignals(String name, TalonFX motor, boolean configureRates) {
    this.motor = motor;
    key = "Hardware/" + name + "/";
    speed = motor.getVelocity();
    current = motor.getStatorCurrent();
    temperature = motor.getDeviceTemp();
    faults = motor.getFaultField();
    stickyFaults = motor.getStickyFaultField();
    fastSignals = new BaseStatusSignal[] {speed, current};
    diagnosticSignals = new BaseStatusSignal[] {temperature, faults, stickyFaults};
    if (configureRates) {
      BaseStatusSignal.setUpdateFrequencyForAll(50, speed, current);
      BaseStatusSignal.setUpdateFrequencyForAll(5, diagnosticSignals);
      motor.optimizeBusUtilization();
    }
  }

  public static MotorSignals mechanism(
      String name, int id, boolean inverted, double kV, double currentLimit) {
    TalonFX motor = new TalonFX(id, CANBus.roboRIO());
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted =
        inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Slot0.kP = 0.1;
    config.Slot0.kV = kV;
    config.CurrentLimits.StatorCurrentLimit = currentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 45;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    boolean configured = motor.getConfigurator().apply(config, 0.15).isOK();
    MotorSignals signals = new MotorSignals(name, motor, true);
    signals.configured = configured;
    return signals;
  }

  public void sample() {
    BaseStatusSignal.refreshAll(false, fastSignals);
  }

  public boolean fresh() {
    return configured && speed.getStatus().isOK() && speed.getTimestamp().getLatency() < 0.12
        && Double.isFinite(rps());
  }

  public double rps() {
    return speed.getValueAsDouble();
  }

  public double amps() {
    return current.getStatus().isOK() && current.getTimestamp().getLatency() < 0.12
        ? current.getValueAsDouble() : Double.NaN;
  }

  public boolean velocity(double rps) {
    commandAccepted = (rps == 0 || !configured ? motor.setControl(neutral)
        : motor.setControl(velocity.withVelocity(rps))).isOK();
    return configured && commandAccepted;
  }

  public boolean voltage(double volts) {
    commandAccepted = (volts == 0 || !configured ? motor.setControl(neutral)
        : motor.setControl(voltage.withOutput(volts))).isOK();
    return configured && commandAccepted;
  }

  public void publishDiagnostics() {
    BaseStatusSignal.refreshAll(false, diagnosticSignals);
    SmartDashboard.putNumber(key + "TemperatureC",
        temperature.getStatus().isOK() ? temperature.getValueAsDouble() : Double.NaN);
    SmartDashboard.putString(key + "CAN", !configured ? "CONFIG" : temperature.getStatus().getName());
    SmartDashboard.putNumber(key + "Faults", faults.getStatus().isOK() ? faults.getValue() : -1);
    SmartDashboard.putNumber(key + "StickyFaults",
        stickyFaults.getStatus().isOK() ? stickyFaults.getValue() : -1);
    SmartDashboard.putBoolean(key + "ControlOK", commandAccepted);
  }
}

