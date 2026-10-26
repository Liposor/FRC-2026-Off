package frc.robot.lib.swerve.config;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import org.junit.jupiter.api.Test;

class SwerveHardwareConfigTest {
  @Test
  void confirmedMk5nR3ValuesStayConsistent() {
    assertEquals(SwerveHardwareConfig.Mk5nRatio.R3, SwerveHardwareConfig.DRIVE_RATIO);
    assertEquals(5.27, SwerveHardwareConfig.DRIVE_RATIO.reduction(), 1e-12);
    assertEquals(5.85216, SwerveHardwareConfig.DRIVE_RATIO.theoreticalFreeSpeed().in(MetersPerSecond), 1e-6);
    assertEquals(2.0, SwerveHardwareConfig.NOMINAL_WHEEL_RADIUS.in(Inches), 1e-12);
    assertEquals(51.79, SwerveHardwareConfig.ROBOT_MASS.in(Kilograms), 1e-12);
  }

  @Test
  void placeholdersNeverUnlockRealHardware() {
    assertFalse(SwerveHardwareConfig.isReadyForRealHardware());
    assertFalse(SwerveHardwareConfig.missingRealHardwareData().isEmpty());
  }
}
