package org.dovershockwave.subsystems.algaerollers.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.dovershockwave.subsystems.algaerollers.AlgaeRollersState;
import org.dovershockwave.subsystems.algaerollers.AlgaeRollersSubsystem;

public class IdleAlgaeRollersCommand extends Command {
  private final AlgaeRollersSubsystem algaeRollers;

  public IdleAlgaeRollersCommand(AlgaeRollersSubsystem algaeRollers) {
    this.algaeRollers = algaeRollers;
    addRequirements(algaeRollers);
  }

  @Override public void execute() {
    if (algaeRollers.getCurrentAmps() > 0.1) {
      algaeRollers.setDesiredState(AlgaeRollersState.IDLE);
    } else {
      algaeRollers.setDesiredState(AlgaeRollersState.STOPPED);
    }
  }
}