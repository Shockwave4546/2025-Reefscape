package org.dovershockwave.commands;

import edu.wpi.first.wpilibj2.command.*;
import org.dovershockwave.ReefScoringPosition;
import org.dovershockwave.ReefScoringSelector;
import org.dovershockwave.subsystems.coralpivot.CoralPivotState;
import org.dovershockwave.subsystems.coralpivot.CoralPivotSubsystem;
import org.dovershockwave.subsystems.coralrollers.CoralRollersSubsystem;
import org.dovershockwave.subsystems.elevator.ElevatorSubsystem;

public class FullScoreCoralCommand extends SequentialCommandGroup {
  public FullScoreCoralCommand(CoralPivotSubsystem coralPivot, CoralRollersSubsystem coralRollers, ElevatorSubsystem elevator, ReefScoringSelector selector) {
    addCommands(
            new ConditionalCommand(new InstantCommand(() -> coralPivot.setDesiredState(CoralPivotState.MOVING_UP)), new InstantCommand(), () -> selector.getLevel() == ReefScoringPosition.ReefLevel.L4),
            new ConditionalCommand(
                    new SequentialCommandGroup(
                            new InstantCommand(() -> elevator.setDesiredState(selector.getLevel()), elevator),
                            new WaitUntilCommand(elevator::atDesiredState),
                            new InstantCommand(() -> coralPivot.setDesiredState(selector.getLevel()), coralPivot),
                            new WaitUntilCommand(coralPivot::atDesiredState),
                            new RunCommand(() -> coralRollers.setDesiredState(selector.getLevel()), coralRollers).withTimeout(0.25)),
                    new SequentialCommandGroup(
                            new InstantCommand(() -> elevator.setDesiredState(selector.getLevel()), elevator),
                            new InstantCommand(() -> coralPivot.setDesiredState(selector.getLevel()), coralPivot),
                            new WaitUntilCommand(elevator::atDesiredState),
                            new WaitUntilCommand(coralPivot::atDesiredState),
                            new RunCommand(() -> coralRollers.setDesiredState(selector.getLevel()), coralRollers).withTimeout(0.25))
                    , () -> selector.getLevel() == ReefScoringPosition.ReefLevel.L4)
    );
  }
}