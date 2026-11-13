package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.ConfigMechanisms;
import frc.robot.config.ConfigMechanisms.Shot;
import frc.robot.util.MotorSignals;

public final class ShooterSubsystem extends SubsystemBase {
  private final MotorSignals right = MotorSignals.mechanism(
      "Shooter24", ConfigMechanisms.RIGHT_SHOOTER_ID, false, ConfigMechanisms.FLYWHEEL_KV, 100);
  private final MotorSignals left = MotorSignals.mechanism(
      "Shooter25", ConfigMechanisms.LEFT_SHOOTER_ID, true, ConfigMechanisms.FLYWHEEL_KV, 100);
  private final SparkFlex flap = new SparkFlex(ConfigMechanisms.FLAP_ID, MotorType.kBrushless);
  private final RelativeEncoder flapEncoder = flap.getEncoder();
  private final SparkClosedLoopController flapControl = flap.getClosedLoopController();
  private final boolean flapConfigured;
  private boolean flapReferenced = RobotBase.isSimulation();
  private boolean flapHealthy;
  private boolean flapCommandAccepted;
  private boolean flywheelCommandAccepted;
  private double flapDegrees;
  private double flapDegreesPerSecond;
  private double targetRpm;
  private double targetFlapDegrees;
  private int flapFaults;

  public ShooterSubsystem() {
    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake).inverted(false)
        .smartCurrentLimit(ConfigMechanisms.FLAP_CURRENT_LIMIT_AMPS);
    config.encoder.positionConversionFactor(360.0 / ConfigMechanisms.FLAP_REDUCTION);
    config.encoder.velocityConversionFactor(6.0 / ConfigMechanisms.FLAP_REDUCTION);
    config.closedLoop.p(0.04).d(0.001)
        .outputRange(-ConfigMechanisms.FLAP_MAX_OUTPUT, ConfigMechanisms.FLAP_MAX_OUTPUT);
    config.softLimit.forwardSoftLimit(ConfigMechanisms.FLAP_MAX_DEGREES).forwardSoftLimitEnabled(true)
        .reverseSoftLimit(0).reverseSoftLimitEnabled(true);
    config.signals.primaryEncoderPositionPeriodMs(20).primaryEncoderVelocityPeriodMs(20)
        .faultsPeriodMs(100).warningsPeriodMs(100).motorTemperaturePeriodMs(200);
    flapConfigured = flap.configure(config, ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters) == REVLibError.kOk;
    flap.setCANTimeout(0);
  }

  @Override
  public void periodic() {
    right.sample();
    left.sample();
    flapDegrees = flapEncoder.getPosition();
    boolean positionOk = flap.getLastError() == REVLibError.kOk;
    flapDegreesPerSecond = flapEncoder.getVelocity();
    boolean velocityOk = flap.getLastError() == REVLibError.kOk;
    var faults = flap.getFaults();
    boolean faultsOk = flap.getLastError() == REVLibError.kOk;
    flapFaults = faults.rawBits;
    boolean valid = flapConfigured && positionOk && velocityOk && faultsOk && !faults.sensor
        && !faults.can && !faults.escEeprom && Double.isFinite(flapDegrees)
        && flapDegrees >= -2 && flapDegrees <= ConfigMechanisms.FLAP_MAX_DEGREES + 2;
    var warnings = flap.getWarnings();
    if (RobotBase.isReal() && (warnings.hasReset || (positionOk && velocityOk
        && (flapDegrees < -2 || flapDegrees > ConfigMechanisms.FLAP_MAX_DEGREES + 2)))) {
      flapReferenced = false;
    }
    flapHealthy = valid || RobotBase.isSimulation();
  }

  public boolean referenceAtMechanicalZero() {
    if (!DriverStation.isDisabled() || !flapConfigured) {
      return false;
    }
    // Posicione o flap no zero físico antes de confirmar no painel. Sem movimento de homing.
    flap.setCANTimeout(30);
    try {
      flapReferenced = flap.clearFaults() == REVLibError.kOk
          && flapEncoder.setPosition(0) == REVLibError.kOk;
    } finally {
      flap.setCANTimeout(0);
    }
    return flapReferenced;
  }

  public void prepare(Shot shot) {
    targetRpm = shot.rpm();
    targetFlapDegrees = shot.flapDegrees();
    boolean flywheelReadyForControl = right.fresh() && left.fresh();
    boolean rightAccepted = right.velocity(flywheelReadyForControl ? shot.rpm() / 60 : 0);
    boolean leftAccepted = left.velocity(flywheelReadyForControl ? shot.rpm() / 60 : 0);
    flywheelCommandAccepted = flywheelReadyForControl && rightAccepted && leftAccepted;
    if (!flywheelCommandAccepted) {
      right.velocity(0);
      left.velocity(0);
    }
    if (flapReferenced && flapHealthy) {
      flapCommandAccepted = flapControl.setSetpoint(shot.flapDegrees(), ControlType.kPosition)
          == REVLibError.kOk;
    } else {
      flap.stopMotor();
      flapCommandAccepted = false;
    }
  }

  public boolean feedbackReady() {
    return right.fresh() && left.fresh() && flapReferenced && flapHealthy;
  }

  public boolean ready(Shot shot) {
    return feedbackReady() && flapCommandAccepted && flywheelCommandAccepted
        && Math.abs(right.rps() * 60 - shot.rpm()) < ConfigMechanisms.RPM_READY_TOLERANCE
        && Math.abs(left.rps() * 60 - shot.rpm()) < ConfigMechanisms.RPM_READY_TOLERANCE
        && Math.abs(flapDegrees - shot.flapDegrees()) < ConfigMechanisms.FLAP_READY_TOLERANCE_DEGREES
        && Math.abs(flapDegreesPerSecond) < 8;
  }

  public double averageRpm() {
    return right.fresh() && left.fresh() ? (right.rps() + left.rps()) * 30 : Double.NaN;
  }

  public double currentAmps() {
    return right.amps() + left.amps();
  }

  public void stop() {
    targetRpm = 0;
    right.velocity(0);
    left.velocity(0);
    flap.stopMotor();
    flapCommandAccepted = false;
    flywheelCommandAccepted = false;
  }

  public void publishDiagnostics() {
    right.publishDiagnostics();
    left.publishDiagnostics();
    double temp = flap.getMotorTemperature();
    SmartDashboard.putNumber("Hardware/Flap61/TemperatureC",
        flap.getLastError() == REVLibError.kOk ? temp : Double.NaN);
    SmartDashboard.putString("Hardware/Flap61/CAN", flapHealthy ? "OK" : "SEM_FEEDBACK");
    SmartDashboard.putNumber("Hardware/Flap61/Faults", flapFaults);
    SmartDashboard.putNumber("Hardware/Flap61/StickyFaults",
        flap.getStickyFaults().rawBits);
    SmartDashboard.putNumber("Shooter/RPM", averageRpm());
    SmartDashboard.putNumber("Shooter/TargetRPM", targetRpm);
    SmartDashboard.putNumber("Shooter/FlapDeg", flapDegrees);
    SmartDashboard.putNumber("Shooter/TargetFlapDeg", targetFlapDegrees);
    SmartDashboard.putBoolean("Shooter/FlapReferenced", flapReferenced);
  }
}

