package org.dovershockwave.subsystems.coralrollers.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import org.dovershockwave.subsystems.coralrollers.CoralRollersState;
import org.dovershockwave.subsystems.coralrollers.CoralRollersSubsystem;

public class IntakeCoralCommand extends Command {
  /**
   * The current (A) at which a coral is considered to be within the end effector.
   */
  private static final double CURRENT_TRIGGER = 7.5;
  /**
   * When initially intaking a coral, the current spikes to reach enough torque, however,
   * afterward it will essentially drop to 0A. A sustained current of {@link #CURRENT_TRIGGER} is
   * required to consider the coral to be within the end effector.
   */
  public final Debouncer currentDebouncer = new Debouncer(0.15);
  public final CoralRollersSubsystem coralRollers;

  public IntakeCoralCommand(CoralRollersSubsystem coralRollers) {
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
    return currentDebouncer.calculate(Math.abs(coralRollers.getCurrentAmps()) > CURRENT_TRIGGER);
  }
}