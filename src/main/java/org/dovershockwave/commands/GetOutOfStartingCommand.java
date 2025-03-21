package org.dovershockwave.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.dovershockwave.subsystems.coralpivot.CoralPivotState;
import org.dovershockwave.subsystems.coralpivot.CoralPivotSubsystem;

public class GetOutOfStartingCommand extends SequentialCommandGroup {
  public GetOutOfStartingCommand(CoralPivotSubsystem coralPivot) {
    addCommands(
            new InstantCommand(() -> coralPivot.setDesiredState(CoralPivotState.SAFE_POSITION_AFTER_START_TWO)),
            new WaitUntilCommand(coralPivot::atDesiredState),
            new InstantCommand(() -> coralPivot.setDesiredState(CoralPivotState.SAFE_POSITION_AFTER_START_THREE)),
            new WaitUntilCommand(coralPivot::atDesiredState),
            new InstantCommand(() -> coralPivot.setDesiredState(CoralPivotState.SAFE_POSITION_AFTER_START_FOUR))
    );
  }
}
