package frc.robot.control;

import java.io.IOException;
import java.nio.file.Files;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.config.FieldTargets;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/** Rotas da tela e retorno à zona de tiro. Todos os destinos são pose no referencial azul. */
public final class Navigation {
  public enum Result { IDLE, RUNNING, ARRIVED, CANCELLED, UNAVAILABLE, BLOCKED, TIMEOUT }
  private static final PathConstraints TRANSIT = new PathConstraints(1.75, 1.5, 2.0, 3.0);
  private static final PathConstraints COLLECT = new PathConstraints(1.3, 1.2, 1.8, 2.5);
  private final CommandSwerveDrivetrain drive;
  private final Map<String, PathPlannerPath> collectPaths = new LinkedHashMap<>();
  private JSONArray grid;
  private double gridStep;
  private Command active;
  private Pose2d target;
  private Result result = Result.IDLE;
  private boolean cancelling;

  public Navigation(CommandSwerveDrivetrain drive) {
    this.drive = drive;
    if (!drive.isPathPlannerConfigured()) {
      return;
    }
    try {
      var file = Filesystem.getDeployDirectory().toPath().resolve("pathplanner/navgrid.json");
      JSONObject data = (JSONObject) new JSONParser().parse(Files.readString(file));
      grid = (JSONArray) data.get("grid");
      gridStep = ((Number) data.get("nodeSizeMeters")).doubleValue();
      if (grid == null || grid.isEmpty() || !Double.isFinite(gridStep) || gridStep <= 0) {
        throw new IOException("Navgrid ausente ou inválida");
      }
    } catch (Exception error) {
      grid = null;
      DriverStation.reportWarning("Navgrid do STORM: " + error.getMessage(), false);
    }
    for (String name : new String[] {"Coleta Sul", "Coleta Norte"}) {
      try {
        PathPlannerPath path = PathPlannerPath.fromPathFile(name);
        if (!pathClear(path) || !pathClear(path.flipPath())) {
          throw new IOException("trajeto cruza célula bloqueada");
        }
        collectPaths.put(name, path);
      } catch (Exception error) {
        DriverStation.reportWarning("Rota " + name + ": " + error.getMessage(), false);
      }
    }
  }

  public boolean active() { return active != null && active.isScheduled(); }
  public Result result() { return result; }
  public Pose2d target() { return active() ? target : null; }
  public Set<String> collectRoutes() { return collectPaths.keySet(); }

  public boolean nearestShootingPoint(Pose2d current, Alliance alliance) {
    Optional<Pose2d> nearest = FieldTargets.shootingPoints(alliance).stream()
        .filter(pose -> clear(pose.getTranslation()))
        .min(Comparator.comparingDouble(pose ->
            current.getTranslation().getDistance(pose.getTranslation())));
    if (nearest.isEmpty()) {
      result = Result.BLOCKED;
      return false;
    }
    return toPoint(current, nearest.get());
  }

  public boolean toPoint(Pose2d current, Pose2d destination) {
    if (!validEndpoints(current, destination)) {
      return false;
    }
    return start(AutoBuilder.pathfindToPose(destination, TRANSIT), destination);
  }

  public boolean collect(Pose2d current, Alliance alliance, String name) {
    PathPlannerPath blue = collectPaths.get(name);
    if (blue == null) {
      result = Result.UNAVAILABLE;
      return false;
    }
    PathPlannerPath path = alliance == Alliance.Red ? blue.flipPath() : blue;
    path.preventFlipping = true;
    Pose2d end = new Pose2d(path.getPoint(path.numPoints() - 1).position,
        path.getGoalEndState().rotation());
    Pose2d start = path.getStartingHolonomicPose().orElseThrow();
    if (!validEndpoints(current, end)) {
      return false;
    }
    if (!clear(start.getTranslation())) {
      result = Result.BLOCKED;
      return false;
    }
    // Perto do início, seguir diretamente evita a trajetória de aproximação degenerada.
    Command route = current.getTranslation().getDistance(start.getTranslation()) < 0.18
        ? AutoBuilder.followPath(path)
        : AutoBuilder.pathfindThenFollowPath(path, COLLECT);
    return start(route, end);
  }

  private boolean validEndpoints(Pose2d current, Pose2d destination) {
    if (!drive.isPathPlannerConfigured() || grid == null) {
      result = Result.UNAVAILABLE;
      return false;
    }
    if (!clear(current.getTranslation()) || !clear(destination.getTranslation())) {
      result = Result.BLOCKED;
      return false;
    }
    return true;
  }

  private boolean start(Command motion, Pose2d destination) {
    cancel();
    target = destination;
    result = Result.RUNNING;
    double[] started = new double[1];
    active = Commands.sequence(Commands.runOnce(() -> started[0] = Timer.getFPGATimestamp()),
        motion).withTimeout(12).finallyDo(interrupted -> {
          drive.brakeNow();
          double error = drive.getState().Pose.getTranslation().getDistance(destination.getTranslation());
          double headingError = Math.abs(drive.getState().Pose.getRotation()
              .minus(destination.getRotation()).getRadians());
          if (cancelling || interrupted) {
            result = Result.CANCELLED;
          } else if (Timer.getFPGATimestamp() - started[0] >= 11.95) {
            result = Result.TIMEOUT;
          } else {
            result = error < 0.25 && headingError < Math.toRadians(12)
                ? Result.ARRIVED : Result.BLOCKED;
          }
        }).withName("Storm/Navigation");
    CommandScheduler.getInstance().schedule(active);
    if (!active.isScheduled()) {
      result = Result.UNAVAILABLE;
      return false;
    }
    return true;
  }

  public void cancel() {
    if (active()) {
      cancelling = true;
      active.cancel();
      cancelling = false;
    }
  }

  private boolean pathClear(PathPlannerPath path) {
    for (var point : path.getAllPathPoints()) {
      if (!clear(point.position)) {
        return false;
      }
    }
    return true;
  }

  private boolean clear(Translation2d point) {
    if (grid == null || !Double.isFinite(point.getX()) || !Double.isFinite(point.getY())
        || point.getX() < FieldTargets.SWEPT_RADIUS || point.getX() > FieldTargets.LENGTH - FieldTargets.SWEPT_RADIUS
        || point.getY() < FieldTargets.SWEPT_RADIUS || point.getY() > FieldTargets.WIDTH - FieldTargets.SWEPT_RADIUS) {
      return false;
    }
    int row = (int) (point.getY() / gridStep);
    int col = (int) (point.getX() / gridStep);
    if (row >= grid.size()) {
      return false;
    }
    JSONArray cells = (JSONArray) grid.get(row);
    return col < cells.size() && Boolean.FALSE.equals(cells.get(col));
  }
}

