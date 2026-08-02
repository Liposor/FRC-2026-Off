package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.ConfigLocalization;
import frc.robot.generated.TunerConstants;

/** Monitora proxies de saude da roboRIO sem adicionar um profiler pesado ao loop principal. */
public final class PerformanceMonitor extends SubsystemBase {
  private double previousPeriodicSeconds = Double.NaN;
  private double maximumLoopIntervalSeconds;
  private double lastPublishSeconds = Double.NEGATIVE_INFINITY;
  private double lastWarningSeconds = Double.NEGATIVE_INFINITY;

  @Override
  public void periodic() {
    double nowSeconds = Timer.getFPGATimestamp();
    if (Double.isFinite(previousPeriodicSeconds)) {
      maximumLoopIntervalSeconds =
          Math.max(maximumLoopIntervalSeconds, nowSeconds - previousPeriodicSeconds);
    }
    previousPeriodicSeconds = nowSeconds;

    if (nowSeconds - lastPublishSeconds
        < ConfigLocalization.PERFORMANCE_TELEMETRY_PERIOD_SECONDS) {
      return;
    }
    lastPublishSeconds = nowSeconds;

    var canStatus = TunerConstants.CAN_BUS.getStatus();
    double cpuTemperature = RobotController.getCPUTemp();
    Runtime runtime = Runtime.getRuntime();
    double usedMemoryMiB =
        (runtime.totalMemory() - runtime.freeMemory()) / (1024.0 * 1024.0);

    SmartDashboard.putNumber("Performance/MaxLoopIntervalMs", maximumLoopIntervalSeconds * 1000.0);
    SmartDashboard.putNumber("Performance/CanUtilizationPercent", canStatus.BusUtilization * 100.0);
    SmartDashboard.putNumber("Performance/CanBusOffCount", canStatus.BusOffCount);
    SmartDashboard.putNumber("Performance/CanTxFullCount", canStatus.TxFullCount);
    SmartDashboard.putNumber("Performance/CanReceiveErrors", canStatus.REC);
    SmartDashboard.putNumber("Performance/CanTransmitErrors", canStatus.TEC);
    SmartDashboard.putNumber("Performance/CpuTemperatureC", cpuTemperature);
    SmartDashboard.putNumber("Performance/JavaUsedMemoryMiB", usedMemoryMiB);

    if (nowSeconds - lastWarningSeconds >= 2.0) {
      if (maximumLoopIntervalSeconds > ConfigLocalization.LOOP_WARNING_SECONDS) {
        DriverStation.reportWarning(
            String.format(
                "Loop principal atrasou: maximo recente %.1f ms",
                maximumLoopIntervalSeconds * 1000.0),
            false);
        lastWarningSeconds = nowSeconds;
      } else if (canStatus.BusUtilization > ConfigLocalization.CAN_WARNING_UTILIZATION) {
        DriverStation.reportWarning(
            String.format("CAN acima do limite saudavel: %.1f%%", canStatus.BusUtilization * 100.0),
            false);
        lastWarningSeconds = nowSeconds;
      } else if (RobotBase.isReal()
          && cpuTemperature > ConfigLocalization.CPU_TEMPERATURE_WARNING_CELSIUS) {
        DriverStation.reportWarning(
            String.format("Temperatura da roboRIO alta: %.1f C", cpuTemperature), false);
        lastWarningSeconds = nowSeconds;
      }
    }
    maximumLoopIntervalSeconds = 0.0;
  }
}
