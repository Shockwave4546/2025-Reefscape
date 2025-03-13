package org.dovershockwave.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.dovershockwave.ReefScoringSelector;
import org.dovershockwave.subsystems.coralpivot.CoralPivotState;
import org.dovershockwave.subsystems.coralpivot.CoralPivotSubsystem;
import org.dovershockwave.subsystems.coralrollers.CoralRollersState;
import org.dovershockwave.subsystems.coralrollers.CoralRollersSubsystem;
import org.dovershockwave.subsystems.elevator.ElevatorState;
import org.dovershockwave.subsystems.elevator.ElevatorSubsystem;

public class FullScoreCoralCopyCommand extends SequentialCommandGroup {
  public FullScoreCoralCopyCommand(CoralPivotSubsystem coralPivot, CoralRollersSubsystem coralRollers, ElevatorSubsystem elevator, ReefScoringSelector selector) {
    addCommands(
            new InstantCommand(() -> elevator.setDesiredState(selector.getLevel()), elevator),
            new WaitUntilCommand(elevator::atDesiredState),
            new InstantCommand(() -> coralPivot.setDesiredState(selector.getLevel()), coralPivot),
            new WaitUntilCommand(coralPivot::atDesiredState),
            new RunCommand(() -> coralRollers.setDesiredState(selector.getLevel()), coralRollers).withTimeout(0.5).finallyDo(() -> {
              coralPivot.setDesiredState(CoralPivotState.MOVING);
              coralRollers.setDesiredState(CoralRollersState.STOPPED);

//              while (!coralPivot.atDesiredState()) {
//                coralPivot.setDesiredState(CoralPivotState.MOVING);
//              }
              elevator.setDesiredState(ElevatorState.STARTING);
            })
    );

    addRequirements(coralPivot, coralRollers, elevator);
  }
}