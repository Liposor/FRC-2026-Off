package frc.robot.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.ConfigVision;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.vision.VisionReliability.Result;

/** Envia orientacao ao MegaTag2, filtra cada frame e funde somente medidas plausiveis. */
public final class VisionSubsystem extends SubsystemBase {
  private final CommandSwerveDrivetrain drivetrain;
  private final List<CameraState> cameras = new ArrayList<>();
  private final StatusSignal<Angle> pitch;
  private final StatusSignal<Angle> roll;
  private final StatusSignal<AngularVelocity> yawRate;
  private final StatusSignal<AngularVelocity> pitchRate;
  private final StatusSignal<AngularVelocity> rollRate;
  private double lastUpdateSeconds = Double.NEGATIVE_INFINITY;
  private double lastTelemetrySeconds = Double.NEGATIVE_INFINITY;

  public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    for (var cameraConfig : ConfigVision.CAMERAS) {
      cameras.add(new CameraState(new LimelightCameraIO(cameraConfig)));
    }

    var pigeon = drivetrain.getPigeon2();
    pitch = pigeon.getPitch();
    roll = pigeon.getRoll();
    yawRate = pigeon.getAngularVelocityZDevice();
    pitchRate = pigeon.getAngularVelocityYDevice();
    rollRate = pigeon.getAngularVelocityXDevice();
    pitch.setUpdateFrequency(50.0);
    roll.setUpdateFrequency(50.0);
    yawRate.setUpdateFrequency(50.0);
    pitchRate.setUpdateFrequency(50.0);
    rollRate.setUpdateFrequency(50.0);
  }

  @Override
  public void periodic() {
    double nowSeconds = Timer.getFPGATimestamp();
    if (nowSeconds - lastUpdateSeconds < ConfigVision.UPDATE_PERIOD_SECONDS) {
      return;
    }
    lastUpdateSeconds = nowSeconds;

    BaseStatusSignal.refreshAll(pitch, roll, yawRate, pitchRate, rollRate);
    double pitchDegrees = pitch.getValue().in(Degrees);
    double rollDegrees = roll.getValue().in(Degrees);
    double yawRateDegreesPerSecond = yawRate.getValue().in(DegreesPerSecond);
    double pitchRateDegreesPerSecond = pitchRate.getValue().in(DegreesPerSecond);
    double rollRateDegreesPerSecond = rollRate.getValue().in(DegreesPerSecond);
    double fieldYawDegrees = drivetrain.getState().Pose.getRotation().getDegrees();

    for (CameraState camera : cameras) {
      camera.io.updateRobotOrientation(
          fieldYawDegrees,
          yawRateDegreesPerSecond,
          pitchDegrees,
          pitchRateDegreesPerSecond,
          rollDegrees,
          rollRateDegreesPerSecond);
      camera.io
          .readLatestObservation()
          .ifPresent(
              observation ->
                  processObservation(
                      camera, observation, nowSeconds, yawRateDegreesPerSecond));
    }

    if (nowSeconds - lastTelemetrySeconds >= ConfigVision.TELEMETRY_PERIOD_SECONDS) {
      lastTelemetrySeconds = nowSeconds;
      publishTelemetry();
    }
  }

  private void processObservation(
      CameraState camera,
      VisionObservation observation,
      double nowSeconds,
      double yawRateDegreesPerSecond) {
    Result result =
        VisionReliability.evaluate(
            observation,
            drivetrain.getState().Pose,
            nowSeconds,
            yawRateDegreesPerSecond,
            DriverStation.isDisabled(),
            ConfigVision.FIELD_LAYOUT);

    camera.lastObservation = observation;
    camera.lastResult = result;
    if (!result.accepted()) {
      camera.rejectedFrames++;
      return;
    }

    drivetrain.addVisionMeasurement(
        observation.robotPose().toPose2d(),
        observation.timestampSeconds(),
        VecBuilder.fill(
            result.xyStdDevMeters(),
            result.xyStdDevMeters(),
            result.thetaStdDevRadians()));
    camera.acceptedFrames++;
  }

  private void publishTelemetry() {
    int totalAccepted = 0;
    int totalRejected = 0;
    for (CameraState camera : cameras) {
      String prefix = "Vision/" + camera.io.config().name() + "/";
      SmartDashboard.putBoolean(prefix + "Accepted", camera.lastResult.accepted());
      SmartDashboard.putString(prefix + "Reason", camera.lastResult.reason().name());
      SmartDashboard.putNumber(prefix + "Confidence", camera.lastResult.confidence());
      SmartDashboard.putNumber(prefix + "AcceptedFrames", camera.acceptedFrames);
      SmartDashboard.putNumber(prefix + "RejectedFrames", camera.rejectedFrames);
      if (camera.lastObservation != null) {
        SmartDashboard.putString(prefix + "VisibleTagIds", camera.lastObservation.tagIdsText());
        SmartDashboard.putNumber(
            prefix + "AverageDistanceMeters",
            camera.lastObservation.averageTagDistanceMeters());
      }
      totalAccepted += camera.acceptedFrames;
      totalRejected += camera.rejectedFrames;
    }
    SmartDashboard.putNumber("Vision/AcceptedFrames", totalAccepted);
    SmartDashboard.putNumber("Vision/RejectedFrames", totalRejected);
  }

  private static final class CameraState {
    final LimelightCameraIO io;
    VisionObservation lastObservation;
    Result lastResult = Result.rejected(VisionReliability.RejectionReason.NO_TAGS);
    int acceptedFrames;
    int rejectedFrames;

    CameraState(LimelightCameraIO io) {
      this.io = io;
    }
  }
}
