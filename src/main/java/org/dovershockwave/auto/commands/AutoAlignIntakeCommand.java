package org.dovershockwave.auto.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.dovershockwave.HumanPlayerStationPosition;
import org.dovershockwave.commands.FullIntakeCoralCommand;
import org.dovershockwave.subsystems.coralpivot.CoralPivotState;
import org.dovershockwave.subsystems.coralpivot.CoralPivotSubsystem;
import org.dovershockwave.subsystems.coralrollers.CoralRollersSubsystem;
import org.dovershockwave.subsystems.coralrollers.commands.IndexCoralCommand;
import org.dovershockwave.subsystems.elevator.ElevatorState;
import org.dovershockwave.subsystems.elevator.ElevatorSubsystem;
import org.dovershockwave.subsystems.swerve.SwerveSubsystem;
import org.dovershockwave.subsystems.vision.commands.AlignToHumanPlayerCommand;

public class AutoAlignIntakeCommand extends SequentialCommandGroup {
  public AutoAlignIntakeCommand(SwerveSubsystem swerve, CoralPivotSubsystem coralPivot, CoralRollersSubsystem coralRollers, ElevatorSubsystem elevator) {
    addCommands(
            new AlignToHumanPlayerCommand(swerve, HumanPlayerStationPosition.HumanPlayerStationSide.FAR),
            new FullIntakeCoralCommand(coralPivot, coralRollers, elevator),
            new ParallelCommandGroup(
                    new InstantCommand(() -> elevator.setDesiredState(ElevatorState.STARTING), elevator),
                    new InstantCommand(() -> coralPivot.setDesiredState(CoralPivotState.MOVING), coralPivot),
                    new IndexCoralCommand(coralRollers)
            )
    );
  }
}