package org.dovershockwave.commands;

import edu.wpi.first.wpilibj2.command.*;
import org.dovershockwave.subsystems.coralpivot.CoralPivotState;
import org.dovershockwave.subsystems.coralpivot.CoralPivotSubsystem;
import org.dovershockwave.subsystems.coralrollers.CoralRollersSubsystem;
import org.dovershockwave.subsystems.coralrollers.commands.IndexCoralCommand;
import org.dovershockwave.subsystems.coralrollers.commands.IntakeCoralCommand;
import org.dovershockwave.subsystems.elevator.ElevatorState;
import org.dovershockwave.subsystems.elevator.ElevatorSubsystem;

public class FullIntakeCoralCommand extends ParallelCommandGroup {
  public FullIntakeCoralCommand(CoralPivotSubsystem coralPivot, CoralRollersSubsystem coralRollers, ElevatorSubsystem elevator) {
    addCommands(
            new InstantCommand(() -> elevator.setDesiredState(ElevatorState.HUMAN_PLAYER)),
            new InstantCommand(() -> coralPivot.setDesiredState(CoralPivotState.HUMAN_PLAYER)),
            new SequentialCommandGroup(
                    new WaitUntilCommand(elevator::atDesiredState),
                    new WaitUntilCommand(coralPivot::atDesiredState),
                    new IntakeCoralCommand(coralRollers),
                    new IndexCoralCommand(coralRollers),
                    new InstantCommand(() -> coralPivot.setDesiredState(CoralPivotState.MOVING), coralPivot)
            )
    );

    addRequirements(coralPivot, coralRollers);
  }
}