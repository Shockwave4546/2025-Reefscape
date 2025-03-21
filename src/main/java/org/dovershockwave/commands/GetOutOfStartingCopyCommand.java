package org.dovershockwave.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.dovershockwave.subsystems.coralpivot.CoralPivotState;
import org.dovershockwave.subsystems.coralpivot.CoralPivotSubsystem;

public class GetOutOfStartingCopyCommand extends SequentialCommandGroup {
  public GetOutOfStartingCopyCommand(CoralPivotSubsystem coralPivot) {
    addCommands(
            new InstantCommand(() -> coralPivot.setDesiredState(new CoralPivotState(CoralPivotState.MOVING_UP.wristPositionRad(), coralPivot.getDesiredState().armPositionRad())), coralPivot),
            new WaitUntilCommand(coralPivot::atDesiredState),
            new InstantCommand(() -> coralPivot.setDesiredState(new CoralPivotState(coralPivot.getDesiredState().wristPositionRad(), CoralPivotState.MOVING_UP.armPositionRad())), coralPivot)
    );
  }
}
