package org.dovershockwave.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import org.dovershockwave.subsystems.algaepivot.AlgaePivotState;
import org.dovershockwave.subsystems.algaepivot.AlgaePivotSubsystem;
import org.dovershockwave.subsystems.algaerollers.AlgaeRollersState;
import org.dovershockwave.subsystems.algaerollers.AlgaeRollersSubsystem;

public class FullIntakeAlgaeAfterCommand extends ParallelCommandGroup {
  public FullIntakeAlgaeAfterCommand(AlgaePivotSubsystem algaePivot, AlgaeRollersSubsystem algaeRollers) {
    addCommands(
            new RunCommand(() -> algaePivot.setDesiredState(AlgaePivotState.INTAKE_AFTER), algaePivot),
            new RunCommand(() -> algaeRollers.setDesiredState(AlgaeRollersState.INTAKE), algaeRollers).withTimeout(2.0)
    );

    addRequirements(algaePivot, algaeRollers);
  }
}