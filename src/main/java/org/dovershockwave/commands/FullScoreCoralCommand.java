package org.dovershockwave.commands;

import edu.wpi.first.wpilibj2.command.*;
import org.dovershockwave.ReefScoringSelector;
import org.dovershockwave.subsystems.coralpivot.CoralPivotSubsystem;
import org.dovershockwave.subsystems.coralrollers.CoralRollersSubsystem;
import org.dovershockwave.subsystems.coralrollers.commands.ScoreCoralCommand;
import org.dovershockwave.subsystems.elevator.ElevatorSubsystem;
import org.dovershockwave.subsystems.swerve.SwerveSubsystem;
import org.dovershockwave.subsystems.vision.VisionSubsystem;
import org.dovershockwave.subsystems.vision.commands.AlignToReefCoralCommand;

// TODO: 2/2/2025 Idk if this sequence will totally work
public class FullScoreCoralCommand extends SequentialCommandGroup {
  public FullScoreCoralCommand(SwerveSubsystem swerve, VisionSubsystem vision, CoralPivotSubsystem coralPivot, CoralRollersSubsystem coralRollers, ElevatorSubsystem elevator, ReefScoringSelector selector) {
    addCommands(
            new ParallelCommandGroup(
                    new InstantCommand(() -> elevator.setDesiredState(selector.getLevel())),
                    new InstantCommand(() -> coralPivot.setDesiredState(selector.getLevel()))
            ),
            new WaitUntilCommand(elevator::atDesiredState),
            new WaitUntilCommand(coralPivot::atDesiredState),
//            new AlignToReefCoralCommand(swerve, vision, selector),
            new ScoreCoralCommand(coralRollers, selector).withTimeout(2)
    );

    addRequirements(swerve, vision, coralPivot, coralRollers, elevator);
  }
}