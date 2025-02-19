package org.dovershockwave.subsystems.coralrollers.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import org.dovershockwave.subsystems.coralrollers.CoralRollersState;
import org.dovershockwave.subsystems.coralrollers.CoralRollersSubsystem;

public class ScoreCoralCommand extends Command {
  /**
   * The currents (A) at which a coral is considered to be out of the end effector.
   */
  private static final double CURRENT_TRIGGER_MIN = -0.5;
  private static final double CURRENT_TRIGGER_MAX = 0.5;
  // TODO: 2/18/2025 Find a better debounceTime
  public final Debouncer currentDebouncer = new Debouncer(0.1);
  public final CoralRollersSubsystem coralRollers;

  public ScoreCoralCommand(CoralRollersSubsystem coralRollers) {
    this.coralRollers = coralRollers;
    addRequirements(coralRollers);
  }

  @Override public void initialize() {
    coralRollers.setDesiredState(CoralRollersState.INTAKE);
  }

  @Override public void execute() {
    coralRollers.setDesiredState(CoralRollersState.INTAKE);
  }

  @Override public void end(boolean interrupted) {
    coralRollers.setDesiredState(CoralRollersState.STOPPED);
  }

  @Override public boolean isFinished() {
    final var current = coralRollers.getCurrentAmps();
    return currentDebouncer.calculate(current >= CURRENT_TRIGGER_MIN && current <= CURRENT_TRIGGER_MAX);
  }
}