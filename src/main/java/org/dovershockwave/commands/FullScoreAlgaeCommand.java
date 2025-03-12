package org.dovershockwave.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import org.dovershockwave.subsystems.algaepivot.AlgaePivotState;
import org.dovershockwave.subsystems.algaepivot.AlgaePivotSubsystem;
import org.dovershockwave.subsystems.algaerollers.AlgaeRollersState;
import org.dovershockwave.subsystems.algaerollers.AlgaeRollersSubsystem;

public class FullScoreAlgaeCommand extends ParallelCommandGroup {
  public FullScoreAlgaeCommand(AlgaePivotSubsystem algaePivot, AlgaeRollersSubsystem algaeRollers) {
    addCommands(
            new RunCommand(() -> algaePivot.setDesiredState(AlgaePivotState.OUTTAKE), algaePivot),
            new RunCommand(() -> algaeRollers.setDesiredState(AlgaeRollersState.OUTTAKE), algaeRollers)
    );

    addRequirements(algaePivot, algaeRollers);
  }
}