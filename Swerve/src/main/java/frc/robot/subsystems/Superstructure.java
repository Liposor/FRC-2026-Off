package frc.robot.subsystems;

import java.util.Objects;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.ConfigSwerve;
import frc.robot.config.ConfigSwerve.DriveProfile;

/**
 * Coordena o estado global do robo e transforma intencoes em comandos.
 *
 * <p>Enquanto os mecanismos ainda nao existem, esta classe ja controla o perfil do drivetrain.
 * Quando intake, shooter, elevador ou climber forem adicionados, seus comandos podem ser
 * encapsulados por {@link #collectCommand(Command)}, {@link #scoreCommand(Command)} e
 * {@link #climbCommand(Command)}.
 */
public class Superstructure extends SubsystemBase {
  public enum Goal {
    IDLE("Ocioso"),
    COLLECTING("Coletando"),
    HOLDING_GAME_PIECE("Com peca"),
    SCORING("Pontuando"),
    CLIMBING("Escalando");

    private final String displayName;

    Goal(String displayName) {
      this.displayName = displayName;
    }

    public String displayName() {
      return displayName;
    }
  }

  private Goal currentGoal = Goal.IDLE;
  private Goal lastPublishedGoal;
  private double lastPublishedSeconds = Double.NEGATIVE_INFINITY;

  public Goal getGoal() {
    return currentGoal;
  }

  /** Retorna o perfil que o comando default do swerve deve usar neste ciclo. */
  public DriveProfile getDriveProfile() {
    return switch (currentGoal) {
      case COLLECTING -> ConfigSwerve.COLLECTING_PROFILE;
      case HOLDING_GAME_PIECE -> ConfigSwerve.HOLDING_PROFILE;
      case SCORING -> ConfigSwerve.SCORING_PROFILE;
      case CLIMBING -> ConfigSwerve.CLIMBING_PROFILE;
      case IDLE -> ConfigSwerve.NORMAL_PROFILE;
    };
  }

  /** Troca o goal uma vez e termina imediatamente. */
  public Command setGoalCommand(Goal goal) {
    Objects.requireNonNull(goal, "goal");
    return runOnce(() -> setGoal(goal)).withName("Superstructure/SetGoal/" + goal.name());
  }

  /**
   * Mantem um goal enquanto o comando estiver agendado e aplica outro quando ele terminar.
   * Ideal para botoes configurados com {@code whileTrue}.
   */
  public Command holdGoalCommand(Goal activeGoal, Goal endGoal) {
    Objects.requireNonNull(activeGoal, "activeGoal");
    Objects.requireNonNull(endGoal, "endGoal");
    return startEnd(() -> setGoal(activeGoal), () -> setGoal(endGoal))
        .withName("Superstructure/Hold/" + activeGoal.name());
  }

  /**
   * Executa uma acao de mecanismo dentro de um goal e garante um estado final mesmo se houver
   * interrupcao.
   */
  public Command runAction(Goal activeGoal, Command mechanismCommand, Goal endGoal) {
    Objects.requireNonNull(mechanismCommand, "mechanismCommand");
    return Commands.sequence(setGoalCommand(activeGoal), mechanismCommand)
        .finallyDo(interrupted -> setGoal(endGoal))
        .withName("Superstructure/Action/" + activeGoal.name());
  }

  /** Exemplo futuro: {@code superstructure.collectCommand(intake.collectCommand())}. */
  public Command collectCommand(Command intakeCommand) {
    return runAction(Goal.COLLECTING, intakeCommand, Goal.HOLDING_GAME_PIECE);
  }

  /** Exemplo futuro: {@code superstructure.scoreCommand(shooter.scoreCommand())}. */
  public Command scoreCommand(Command scoringCommand) {
    return runAction(Goal.SCORING, scoringCommand, Goal.IDLE);
  }

  /** Exemplo futuro: {@code superstructure.climbCommand(climber.climbCommand())}. */
  public Command climbCommand(Command climbingCommand) {
    return runAction(Goal.CLIMBING, climbingCommand, Goal.CLIMBING);
  }

  private void setGoal(Goal newGoal) {
    currentGoal = Objects.requireNonNull(newGoal, "newGoal");
  }

  @Override
  public void periodic() {
    double nowSeconds = Timer.getFPGATimestamp();
    if (currentGoal == lastPublishedGoal && nowSeconds - lastPublishedSeconds < 0.10) {
      return;
    }
    lastPublishedGoal = currentGoal;
    lastPublishedSeconds = nowSeconds;
    DriveProfile profile = getDriveProfile();
    SmartDashboard.putString("Superstructure/Goal", currentGoal.displayName());
    SmartDashboard.putString("Superstructure/DriveProfile", profile.name());
    SmartDashboard.putNumber(
        "Superstructure/TranslationScale", profile.translationScale());
    SmartDashboard.putNumber("Superstructure/RotationScale", profile.rotationScale());
  }
}
