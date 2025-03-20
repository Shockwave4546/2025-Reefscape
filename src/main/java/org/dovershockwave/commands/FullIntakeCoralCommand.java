package org.dovershockwave.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.dovershockwave.subsystems.coralpivot.CoralPivotState;
import org.dovershockwave.subsystems.coralpivot.CoralPivotSubsystem;
import org.dovershockwave.subsystems.coralrollers.CoralRollersSubsystem;
import org.dovershockwave.subsystems.coralrollers.commands.IndexCoralCommand;
import org.dovershockwave.subsystems.coralrollers.commands.IntakeCoralCommand;
import org.dovershockwave.subsystems.elevator.ElevatorState;
import org.dovershockwave.subsystems.elevator.ElevatorSubsystem;

public class FullIntakeCoralCommand extends SequentialCommandGroup {
  public FullIntakeCoralCommand(CoralPivotSubsystem coralPivot, CoralRollersSubsystem coralRollers, ElevatorSubsystem elevator) {
    addCommands(
            new ParallelCommandGroup(
                    new IntakeCoralCommand(coralRollers),
                    new InstantCommand(() -> elevator.setDesiredState(ElevatorState.HUMAN_PLAYER), elevator),
                    new InstantCommand(() -> coralPivot.setDesiredState(CoralPivotState.HUMAN_PLAYER), coralPivot)
            ),
            new WaitUntilCommand(elevator::atDesiredState),
            new WaitUntilCommand(coralPivot::atDesiredState),
            new IndexCoralCommand(coralRollers)
//            new ParallelCommandGroup(
//                    new IndexCoralCommand(coralRollers),
//                    new InstantCommand(() -> elevator.setDesiredState(ElevatorState.STARTING), elevator),
//                    new InstantCommand(() -> coralPivot.setDesiredState(CoralPivotState.MOVING), coralPivot)
//            )
    );
  }
}