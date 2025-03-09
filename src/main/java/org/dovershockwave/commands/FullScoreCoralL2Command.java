package org.dovershockwave.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.dovershockwave.ReefScoringSelector;
import org.dovershockwave.subsystems.coralpivot.CoralPivotState;
import org.dovershockwave.subsystems.coralpivot.CoralPivotSubsystem;
import org.dovershockwave.subsystems.coralrollers.CoralRollersSubsystem;
import org.dovershockwave.subsystems.coralrollers.commands.ScoreCoralCommand;
import org.dovershockwave.subsystems.elevator.ElevatorState;
import org.dovershockwave.subsystems.elevator.ElevatorSubsystem;
import org.dovershockwave.subsystems.swerve.SwerveSubsystem;
import org.dovershockwave.subsystems.vision.VisionSubsystem;

public class FullScoreCoralL2Command extends SequentialCommandGroup {
  public FullScoreCoralL2Command(SwerveSubsystem swerve, VisionSubsystem vision, CoralPivotSubsystem coralPivot, CoralRollersSubsystem coralRollers, ElevatorSubsystem elevator, ReefScoringSelector selector) {
    addCommands(
            new InstantCommand(() -> elevator.setDesiredState(selector.getLevel()), elevator),
            new WaitUntilCommand(() -> elevator.getState().positionRad() >= 52.5),
            new InstantCommand(() -> coralPivot.setDesiredState(selector.getLevel()), coralPivot),
            new WaitUntilCommand(elevator::atDesiredState),
            new WaitUntilCommand(coralPivot::atDesiredState),
            new ScoreCoralCommand(coralRollers, selector).withTimeout(0.5),
            new InstantCommand(() -> coralPivot.setDesiredState(CoralPivotState.MOVING), coralPivot),
            new InstantCommand(() -> elevator.setDesiredState(ElevatorState.STARTING), elevator)
    );

    addRequirements(swerve, vision, coralPivot, coralRollers, elevator);
  }
}