package org.dovershockwave.subsystems.coralrollers.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.dovershockwave.ReefScoringSelector;
import org.dovershockwave.subsystems.coralrollers.CoralRollersState;
import org.dovershockwave.subsystems.coralrollers.CoralRollersSubsystem;

public class ScoreCoralCommand extends Command {
  private final CoralRollersSubsystem coralRollers;
  private final ReefScoringSelector selector;

  public ScoreCoralCommand(CoralRollersSubsystem coralRollers, ReefScoringSelector selector) {
    this.coralRollers = coralRollers;
    this.selector = selector;
    addRequirements(coralRollers);
  }

  @Override public void initialize() {
    coralRollers.setDesiredState(selector.getLevel());
  }

  @Override public void execute() {
    coralRollers.setDesiredState(selector.getLevel());
  }

  @Override public void end(boolean interrupted) {
    coralRollers.setDesiredState(CoralRollersState.STOPPED);
  }
}