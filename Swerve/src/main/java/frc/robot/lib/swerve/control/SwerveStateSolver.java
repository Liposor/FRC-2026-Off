package frc.robot.lib.swerve.control;

import frc.robot.lib.swerve.config.ConfigLocalization;
import frc.robot.subsystems.superstructure.Superstructure.Goal;

/** Resolve prioridades do swerve sem misturar regras de seguranca com leitura de joystick. */
public final class SwerveStateSolver {
  public enum DriveState {
    NORMAL,
    SCORE_X_LOCK,
    COLLISION_X_LOCK
  }

  public DriveState solve(Goal superstructureGoal, boolean collisionBlocked) {
    if (collisionBlocked) {
      return DriveState.COLLISION_X_LOCK;
    }
    if (ConfigLocalization.LOCK_X_WHILE_SCORING
        && (superstructureGoal == Goal.READY_TO_SCORE
            || superstructureGoal == Goal.SCORING
            || superstructureGoal == Goal.SCORE_COMPLETE)) {
      return DriveState.SCORE_X_LOCK;
    }
    return DriveState.NORMAL;
  }
}
