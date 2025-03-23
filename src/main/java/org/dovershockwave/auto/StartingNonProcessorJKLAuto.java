package org.dovershockwave.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.dovershockwave.ReefScoringPosition;
import org.dovershockwave.ReefScoringSelector;
import org.dovershockwave.auto.commands.AutoAlignIntakeCommand;
import org.dovershockwave.auto.commands.AutoScoreCoralL4Command;
import org.dovershockwave.subsystems.coralpivot.CoralPivotState;
import org.dovershockwave.subsystems.coralpivot.CoralPivotSubsystem;
import org.dovershockwave.subsystems.coralrollers.CoralRollersSubsystem;
import org.dovershockwave.subsystems.elevator.ElevatorSubsystem;
import org.dovershockwave.subsystems.swerve.SwerveSubsystem;

public class StartingNonProcessorJKLAuto extends SequentialCommandGroup {
  public StartingNonProcessorJKLAuto(SwerveSubsystem swerve, CoralPivotSubsystem coralPivot, CoralRollersSubsystem coralRollers, ElevatorSubsystem elevator, ReefScoringSelector selector) {
    addCommands(
            swerve.resetOdomThenFollowPath("StartNonProcessor-J"),
            new AutoScoreCoralL4Command(swerve, coralPivot, coralRollers, elevator, selector, ReefScoringPosition.ReefScoringSide.RIGHT),
            swerve.pathFindThenFollowPath("J-HumanNonProcessor"),
            new InstantCommand(() -> coralPivot.setDesiredState(CoralPivotState.MOVING), coralPivot),
            new AutoAlignIntakeCommand(swerve, coralPivot, coralRollers, elevator),
            swerve.pathFindThenFollowPath("HumanNonProcessor-K"),
            new AutoScoreCoralL4Command(swerve, coralPivot, coralRollers, elevator, selector, ReefScoringPosition.ReefScoringSide.LEFT),
            swerve.pathFindThenFollowPath("K-HumanNonProcessor"),
            new InstantCommand(() -> coralPivot.setDesiredState(CoralPivotState.MOVING), coralPivot),
            new AutoAlignIntakeCommand(swerve, coralPivot, coralRollers, elevator),
            swerve.pathFindThenFollowPath("HumanNonProcessor-L"),
            new AutoScoreCoralL4Command(swerve, coralPivot, coralRollers, elevator, selector, ReefScoringPosition.ReefScoringSide.RIGHT)
    );
  }
}
