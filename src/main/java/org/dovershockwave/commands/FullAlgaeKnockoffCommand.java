package org.dovershockwave.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.dovershockwave.ReefScoringPosition;
import org.dovershockwave.ReefScoringSelector;
import org.dovershockwave.subsystems.coralpivot.CoralPivotState;
import org.dovershockwave.subsystems.coralpivot.CoralPivotSubsystem;
import org.dovershockwave.subsystems.coralrollers.CoralRollersState;
import org.dovershockwave.subsystems.coralrollers.CoralRollersSubsystem;
import org.dovershockwave.subsystems.elevator.ElevatorState;
import org.dovershockwave.subsystems.elevator.ElevatorSubsystem;

public class FullAlgaeKnockoffCommand extends ParallelCommandGroup {
  public FullAlgaeKnockoffCommand(CoralPivotSubsystem coralPivot, ElevatorSubsystem elevator, CoralRollersSubsystem coralRollers, ReefScoringSelector selector) {
    addCommands(
            new ConditionalCommand(
                    new InstantCommand(() -> coralPivot.setDesiredState(CoralPivotState.ALGAE_KNOCK_OFF_L2), coralPivot),
                    new InstantCommand(() -> coralPivot.setDesiredState(CoralPivotState.ALGAE_KNOCK_OFF_L3), coralPivot),
                    () -> selector.getLevel() == ReefScoringPosition.ReefLevel.L2
            ),
            new ConditionalCommand(
                    new InstantCommand(() -> elevator.setDesiredState(ElevatorState.KNOCKOFF_ALGAE_L2), elevator),
                    new InstantCommand(() -> elevator.setDesiredState(ElevatorState.KNOCKOFF_ALGAE_L3)),
                    () -> selector.getLevel() == ReefScoringPosition.ReefLevel.L2
            ),

            new InstantCommand(() -> coralRollers.setDesiredState(CoralRollersState.KNOCKOFF_ALGAE), coralRollers)
    );

    addRequirements(coralPivot, elevator);
  }
}