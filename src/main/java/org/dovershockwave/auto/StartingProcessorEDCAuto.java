package org.dovershockwave.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.dovershockwave.ReefScoringPosition;
import org.dovershockwave.ReefScoringSelector;
import org.dovershockwave.auto.commands.AutoAlignIntakeCommand;
import org.dovershockwave.auto.commands.AutoScoreCoralL4Command;
import org.dovershockwave.subsystems.coralpivot.CoralPivotSubsystem;
import org.dovershockwave.subsystems.coralrollers.CoralRollersSubsystem;
import org.dovershockwave.subsystems.elevator.ElevatorSubsystem;
import org.dovershockwave.subsystems.swerve.SwerveSubsystem;

public class StartingProcessorEDCAuto extends SequentialCommandGroup {
  public StartingProcessorEDCAuto(SwerveSubsystem swerve, CoralPivotSubsystem coralPivot, CoralRollersSubsystem coralRollers, ElevatorSubsystem elevator, ReefScoringSelector selector) {
    addCommands(
            swerve.resetOdomThenFollowPath("StartProcessor-E"),
            new AutoScoreCoralL4Command(swerve, coralPivot, coralRollers, elevator, selector, ReefScoringPosition.ReefScoringSide.RIGHT),
            swerve.pathFindThenFollowPath("E-HumanProcessor"),
            new AutoAlignIntakeCommand(swerve, coralPivot, coralRollers, elevator),
            swerve.pathFindThenFollowPath("HumanProcessor-D"),
            new AutoScoreCoralL4Command(swerve, coralPivot, coralRollers, elevator, selector, ReefScoringPosition.ReefScoringSide.LEFT),
            swerve.pathFindThenFollowPath("D-HumanProcessor"),
            new AutoAlignIntakeCommand(swerve, coralPivot, coralRollers, elevator),
            swerve.pathFindThenFollowPath("HumanProcessor-C"),
            new AutoScoreCoralL4Command(swerve, coralPivot, coralRollers, elevator, selector, ReefScoringPosition.ReefScoringSide.RIGHT)
    );
  }
}
