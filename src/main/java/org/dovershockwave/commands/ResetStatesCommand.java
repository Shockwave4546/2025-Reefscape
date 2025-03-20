package org.dovershockwave.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.dovershockwave.subsystems.algaepivot.AlgaePivotState;
import org.dovershockwave.subsystems.algaepivot.AlgaePivotSubsystem;
import org.dovershockwave.subsystems.algaerollers.AlgaeRollersState;
import org.dovershockwave.subsystems.algaerollers.AlgaeRollersSubsystem;
import org.dovershockwave.subsystems.coralpivot.CoralPivotState;
import org.dovershockwave.subsystems.coralpivot.CoralPivotSubsystem;
import org.dovershockwave.subsystems.coralrollers.CoralRollersState;
import org.dovershockwave.subsystems.coralrollers.CoralRollersSubsystem;
import org.dovershockwave.subsystems.elevator.ElevatorState;
import org.dovershockwave.subsystems.elevator.ElevatorSubsystem;

public class ResetStatesCommand extends ParallelCommandGroup {
  public ResetStatesCommand(CoralRollersSubsystem coralRollers, ElevatorSubsystem elevator, CoralPivotSubsystem coralPivot, AlgaePivotSubsystem algaePivot, AlgaeRollersSubsystem algaeRollers) {
    addCommands(
            new InstantCommand(() -> coralRollers.setDesiredState(CoralRollersState.STOPPED), coralRollers),
            new InstantCommand(() -> elevator.setDesiredState(ElevatorState.STARTING), elevator),
            new InstantCommand(() -> algaePivot.setDesiredState(AlgaePivotState.STARTING), algaePivot),
            new InstantCommand(() -> algaeRollers.setDesiredState(AlgaeRollersState.STOPPED), algaeRollers),

            new SequentialCommandGroup(
                    new InstantCommand(() -> coralPivot.setDesiredState(new CoralPivotState(CoralPivotState.MOVING_UP.wristPositionRad(), coralPivot.getDesiredState().armPositionRad())), coralPivot),
                    new WaitUntilCommand(coralPivot::atDesiredState),
                    new InstantCommand(() -> coralPivot.setDesiredState(new CoralPivotState(coralPivot.getDesiredState().wristPositionRad(), CoralPivotState.MOVING_UP.armPositionRad())), coralPivot)
            )
    );
  }
}