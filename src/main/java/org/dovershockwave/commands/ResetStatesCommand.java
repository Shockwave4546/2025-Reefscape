package org.dovershockwave.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.dovershockwave.subsystems.coralpivot.CoralPivotState;
import org.dovershockwave.subsystems.coralpivot.CoralPivotSubsystem;
import org.dovershockwave.subsystems.coralrollers.CoralRollersState;
import org.dovershockwave.subsystems.coralrollers.CoralRollersSubsystem;
import org.dovershockwave.subsystems.elevator.ElevatorState;
import org.dovershockwave.subsystems.elevator.ElevatorSubsystem;

public class ResetStatesCommand extends ParallelCommandGroup {
  public ResetStatesCommand(CoralRollersSubsystem coralRollers, ElevatorSubsystem elevator, CoralPivotSubsystem coralPivot) {
    addCommands(
            new InstantCommand(() -> coralRollers.setDesiredState(CoralRollersState.STOPPED), coralRollers),
            new InstantCommand(() -> elevator.setDesiredState(ElevatorState.STARTING), elevator),
            new InstantCommand(() -> coralPivot.setDesiredState(CoralPivotState.MOVING), coralPivot)
    );

    addRequirements(coralRollers, elevator, coralPivot);
  }
}
