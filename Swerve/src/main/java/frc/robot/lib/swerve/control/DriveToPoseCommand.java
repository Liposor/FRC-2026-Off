package frc.robot.lib.swerve.control;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.swerve.config.ConfigLocalization;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/** Reusable final approach command. Targets use WPILib's blue-origin field frame. */
public final class DriveToPoseCommand extends Command {
  private final SwerveSubsystem drivetrain;
  private final Supplier<Pose2d> targetSupplier;
  private final DriveToPoseController controller = new DriveToPoseController();
  private final SwerveRequest.ApplyRobotSpeeds request =
      new SwerveRequest.ApplyRobotSpeeds().withDriveRequestType(DriveRequestType.Velocity);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private double readyStartSeconds = Double.NaN;

  public DriveToPoseCommand(SwerveSubsystem drivetrain, Supplier<Pose2d> targetSupplier) {
    this.drivetrain = drivetrain;
    this.targetSupplier = targetSupplier;
    addRequirements(drivetrain);
    setName("Swerve/DriveToPose");
  }

  @Override
  public void initialize() {
    controller.reset(drivetrain.getState().Pose, drivetrain.getState().Speeds);
    readyStartSeconds = Double.NaN;
  }

  @Override
  public void execute() {
    Pose2d target = targetSupplier.get();
    var state = drivetrain.getState();
    var requestedSpeeds = controller.calculate(state.Pose, target);
    drivetrain.observeRequestedSpeeds(requestedSpeeds);
    drivetrain.setControl(request.withSpeeds(requestedSpeeds));
    boolean ready = DriveToPoseController.isPreciselyReady(state.Pose, state.Speeds, target);
    if (!ready) readyStartSeconds = Double.NaN;
    else if (!Double.isFinite(readyStartSeconds)) readyStartSeconds = Timer.getFPGATimestamp();
  }

  @Override
  public boolean isFinished() {
    return Double.isFinite(readyStartSeconds)
        && Timer.getFPGATimestamp() - readyStartSeconds
            >= ConfigLocalization.ALIGN_READY_DEBOUNCE_SECONDS;
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.observeRequestedSpeeds(new edu.wpi.first.math.kinematics.ChassisSpeeds());
    drivetrain.setControl(brake);
  }
}
