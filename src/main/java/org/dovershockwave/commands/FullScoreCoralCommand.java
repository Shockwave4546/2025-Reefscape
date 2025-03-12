package org.dovershockwave.commands;

import edu.wpi.first.wpilibj2.command.*;
import org.dovershockwave.ReefScoringPosition;
import org.dovershockwave.ReefScoringSelector;
import org.dovershockwave.subsystems.coralpivot.CoralPivotState;
import org.dovershockwave.subsystems.coralpivot.CoralPivotSubsystem;
import org.dovershockwave.subsystems.coralrollers.CoralRollersState;
import org.dovershockwave.subsystems.coralrollers.CoralRollersSubsystem;
import org.dovershockwave.subsystems.elevator.ElevatorState;
import org.dovershockwave.subsystems.elevator.ElevatorSubsystem;

public class FullScoreCoralCommand extends SequentialCommandGroup {
  public FullScoreCoralCommand(CoralPivotSubsystem coralPivot, CoralRollersSubsystem coralRollers, ElevatorSubsystem elevator, ReefScoringSelector selector) {
    addCommands(
            new InstantCommand(() -> elevator.setDesiredState(selector.getLevel()), elevator),
            new WaitUntilCommand(elevator::atDesiredState),
            new InstantCommand(() -> coralPivot.setDesiredState(selector.getLevel()), coralPivot),
            new WaitUntilCommand(coralPivot::atDesiredState),
            new ParallelCommandGroup(
                    new ConditionalCommand(new WaitCommand(0.5).andThen(new InstantCommand(() -> coralPivot.setDesiredState(CoralPivotState.L2_L3_OUTTAKE), coralPivot)), new InstantCommand(), () -> selector.getLevel() == ReefScoringPosition.ReefLevel.L2 || selector.getLevel() == ReefScoringPosition.ReefLevel.L3),
                    new RunCommand(() -> coralRollers.setDesiredState(selector.getLevel()), coralRollers)
            ).withTimeout(1),
            new InstantCommand(() -> coralPivot.setDesiredState(CoralPivotState.MOVING), coralPivot),
            new InstantCommand(() -> coralRollers.setDesiredState(CoralRollersState.STOPPED), coralRollers),
            new WaitUntilCommand(coralPivot::atDesiredState),
            new InstantCommand(() -> elevator.setDesiredState(ElevatorState.STARTING), elevator)
    );

    addRequirements(coralPivot, coralRollers, elevator);
  }
}