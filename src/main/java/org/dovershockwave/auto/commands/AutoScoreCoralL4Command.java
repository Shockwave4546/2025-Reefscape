package org.dovershockwave.auto.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.dovershockwave.ReefScoringPosition;
import org.dovershockwave.ReefScoringSelector;
import org.dovershockwave.subsystems.coralpivot.CoralPivotState;
import org.dovershockwave.subsystems.coralpivot.CoralPivotSubsystem;
import org.dovershockwave.subsystems.coralrollers.CoralRollersState;
import org.dovershockwave.subsystems.coralrollers.CoralRollersSubsystem;
import org.dovershockwave.subsystems.elevator.ElevatorState;
import org.dovershockwave.subsystems.elevator.ElevatorSubsystem;
import org.dovershockwave.subsystems.swerve.SwerveSubsystem;

public class AutoScoreCoralL4Command extends SequentialCommandGroup {
  public AutoScoreCoralL4Command(SwerveSubsystem swerve, CoralPivotSubsystem coralPivot, CoralRollersSubsystem coralRollers, ElevatorSubsystem elevator, ReefScoringSelector selector, ReefScoringPosition.ReefScoringSide side) {
    addCommands(
            new InstantCommand(() -> coralPivot.setDesiredState(CoralPivotState.MOVING_UP)),

            /*
            Get to position a little offset
             */
            new AlignToReefCoralL4IntermediateCommand(swerve, side, selector),

            /*
            Prep subsystems for scoring
             */
            new InstantCommand(() -> elevator.setDesiredState(ReefScoringPosition.ReefLevel.L4), elevator),
            new WaitUntilCommand(elevator::atDesiredState),
            new InstantCommand(() -> coralPivot.setDesiredState(CoralPivotState.L4), coralPivot),

            /*
            Get to the final position
             */
            new AlignToReefCoralL4Command(swerve, selector, side),

            /*
            Score the coral
             */
            new RunCommand(() -> coralRollers.setDesiredState(ReefScoringPosition.ReefLevel.L4), coralRollers).withTimeout(0.5),

            /*
            Stow all the subsystems
             */
            new InstantCommand(() -> coralPivot.setDesiredState(CoralPivotState.MOVING_UP)),
            new InstantCommand(() -> coralRollers.setDesiredState(CoralRollersState.STOPPED), coralRollers),
            new WaitUntilCommand(coralPivot::atDesiredState),
            new InstantCommand(() -> elevator.setDesiredState(ElevatorState.STARTING), elevator),
            new WaitUntilCommand(elevator::atDesiredState)
    );
  }
}