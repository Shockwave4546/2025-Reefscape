package org.dovershockwave.commands;

import edu.wpi.first.wpilibj2.command.*;
import org.dovershockwave.ReefScoringPosition;
import org.dovershockwave.subsystems.coralpivot.CoralPivotState;
import org.dovershockwave.subsystems.coralpivot.CoralPivotSubsystem;
import org.dovershockwave.subsystems.coralrollers.CoralRollersState;
import org.dovershockwave.subsystems.coralrollers.CoralRollersSubsystem;
import org.dovershockwave.subsystems.elevator.ElevatorState;
import org.dovershockwave.subsystems.elevator.ElevatorSubsystem;

public class AutoFullScoreCoralL4Command extends SequentialCommandGroup {
  public AutoFullScoreCoralL4Command(CoralPivotSubsystem coralPivot, CoralRollersSubsystem coralRollers, ElevatorSubsystem elevator) {
    addCommands(
            new InstantCommand(() -> coralPivot.setDesiredState(CoralPivotState.MOVING_UP), coralPivot),
            new InstantCommand(() -> elevator.setDesiredState(ReefScoringPosition.ReefLevel.L4), elevator),
            new WaitUntilCommand(elevator::atDesiredState),
            new InstantCommand(() -> coralPivot.setDesiredState(ReefScoringPosition.ReefLevel.L4), coralPivot),
            new RunCommand(() -> coralRollers.setDesiredState(ReefScoringPosition.ReefLevel.L4), coralRollers).withTimeout(0.25),
            new ParallelCommandGroup(
                    new InstantCommand(() -> coralPivot.setDesiredState(CoralPivotState.MOVING_UP)),
                    new InstantCommand(() -> coralRollers.setDesiredState(CoralRollersState.STOPPED), coralRollers)
            ),
            new WaitUntilCommand(coralPivot::atDesiredState),
            new InstantCommand(() -> elevator.setDesiredState(ElevatorState.STARTING), elevator)
    );
  }
}