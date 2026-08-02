package frc.robot.lib.swerve.simulation;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

/** CTRE device bridge adapted from the official MapleSim CTRE swerve template. */
public final class MapleSimSwerveDrivetrain {
  private final Pigeon2SimState pigeonSim;
  private final SimSwerveModule[] simModules;
  private final SwerveDriveSimulation mapleSimDrive;

  public MapleSimSwerveDrivetrain(
      Time simPeriod,
      Mass robotMassWithBumpers,
      Distance bumperLengthX,
      Distance bumperWidthY,
      DCMotor driveMotorModel,
      DCMotor steerMotorModel,
      double wheelCoefficientOfFriction,
      Translation2d[] moduleLocations,
      Pigeon2 pigeon,
      SwerveModule<TalonFX, TalonFX, CANcoder>[] modules,
      SwerveModuleConstants<?, ?, ?>... moduleConstants) {
    pigeonSim = pigeon.getSimState();
    simModules = new SimSwerveModule[moduleConstants.length];
    var simulationConfig =
        DriveTrainSimulationConfig.Default()
            .withRobotMass(robotMassWithBumpers)
            .withBumperSize(bumperLengthX, bumperWidthY)
            .withGyro(COTS.ofPigeon2())
            .withCustomModuleTranslations(moduleLocations)
            .withSwerveModule(
                new SwerveModuleSimulationConfig(
                    driveMotorModel,
                    steerMotorModel,
                    moduleConstants[0].DriveMotorGearRatio,
                    moduleConstants[0].SteerMotorGearRatio,
                    Volts.of(moduleConstants[0].DriveFrictionVoltage),
                    Volts.of(moduleConstants[0].SteerFrictionVoltage),
                    Meters.of(moduleConstants[0].WheelRadius),
                    KilogramSquareMeters.of(moduleConstants[0].SteerInertia),
                    wheelCoefficientOfFriction));
    mapleSimDrive = new SwerveDriveSimulation(simulationConfig, Pose2d.kZero);

    SwerveModuleSimulation[] moduleSimulations = mapleSimDrive.getModules();
    for (int i = 0; i < simModules.length; i++) {
      simModules[i] = new SimSwerveModule(moduleSimulations[i], modules[i]);
    }
    SimulatedArena.overrideSimulationTimings(simPeriod, 1);
    SimulatedArena.getInstance().addDriveTrainSimulation(mapleSimDrive);
  }

  public void update() {
    SimulatedArena.getInstance().simulationPeriodic();
    pigeonSim.setRawYaw(mapleSimDrive.getSimulatedDriveTrainPose().getRotation().getMeasure());
    pigeonSim.setAngularVelocityZ(
        RadiansPerSecond.of(
            mapleSimDrive
                .getDriveTrainSimulatedChassisSpeedsRobotRelative()
                .omegaRadiansPerSecond));
  }

  public Pose2d getSimulatedPose() {
    return mapleSimDrive.getSimulatedDriveTrainPose();
  }

  private static final class SimSwerveModule {
    SimSwerveModule(
        SwerveModuleSimulation simulation,
        SwerveModule<TalonFX, TalonFX, CANcoder> module) {
      simulation.useDriveMotorController(new TalonFxMotorControllerSim(module.getDriveMotor()));
      simulation.useSteerMotorController(
          new TalonFxWithRemoteCancoderSim(module.getSteerMotor(), module.getEncoder()));
    }
  }

  private static class TalonFxMotorControllerSim implements SimulatedMotorController {
    private final TalonFXSimState simState;

    TalonFxMotorControllerSim(TalonFX talonFX) {
      simState = talonFX.getSimState();
    }

    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      simState.setRawRotorPosition(encoderAngle);
      simState.setRotorVelocity(encoderVelocity);
      simState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
      return simState.getMotorVoltageMeasure();
    }
  }

  private static final class TalonFxWithRemoteCancoderSim extends TalonFxMotorControllerSim {
    private final CANcoderSimState cancoderSim;

    TalonFxWithRemoteCancoderSim(TalonFX talonFX, CANcoder cancoder) {
      super(talonFX);
      cancoderSim = cancoder.getSimState();
    }

    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      cancoderSim.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
      cancoderSim.setRawPosition(
          mechanismAngle.plus(
              edu.wpi.first.units.Units.Rotations.of(
                  SimulationFaults.cancoderOffsetRotations())));
      cancoderSim.setVelocity(mechanismVelocity);
      return super.updateControlSignal(
          mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
    }
  }

  /** Applies template-only simulation corrections; it is a no-op on real hardware. */
  public static SwerveModuleConstants<?, ?, ?>[] regulateForSimulation(
      SwerveModuleConstants<?, ?, ?>[] constants) {
    if (RobotBase.isReal()) return constants;
    for (SwerveModuleConstants<?, ?, ?> module : constants) {
      module.withEncoderOffset(0)
          .withDriveMotorInverted(false)
          .withSteerMotorInverted(false)
          .withEncoderInverted(false)
          .withSteerMotorGains(
              new Slot0Configs()
                  .withKP(70)
                  .withKI(0)
                  .withKD(4.5)
                  .withKS(0)
                  .withKV(1.91)
                  .withKA(0)
                  .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign))
          .withDriveFrictionVoltage(Volts.of(0.1))
          .withSteerFrictionVoltage(Volts.of(0.05))
          .withSteerInertia(KilogramSquareMeters.of(0.05));
    }
    return constants;
  }
}
