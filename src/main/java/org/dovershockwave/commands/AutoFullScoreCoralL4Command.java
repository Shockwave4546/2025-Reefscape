package org.dovershockwave.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.dovershockwave.subsystems.coralpivot.CoralPivotState;
import org.dovershockwave.subsystems.coralpivot.CoralPivotSubsystem;
import org.dovershockwave.subsystems.coralrollers.CoralRollersState;
import org.dovershockwave.subsystems.coralrollers.CoralRollersSubsystem;
import org.dovershockwave.subsystems.elevator.ElevatorState;
import org.dovershockwave.subsystems.elevator.ElevatorSubsystem;

public class AutoFullScoreCoralL4Command extends SequentialCommandGroup {
  public AutoFullScoreCoralL4Command(CoralPivotSubsystem coralPivot, CoralRollersSubsystem coralRollers, ElevatorSubsystem elevator) {
        addCommands(
            new InstantCommand(() -> elevator.setDesiredState(ElevatorState.L4), elevator),
            new WaitUntilCommand(elevator::atDesiredState),
            new InstantCommand(() -> coralPivot.setDesiredState(CoralPivotState.L4), coralPivot),
            new WaitUntilCommand(coralPivot::atDesiredState),
            new RunCommand(() -> coralRollers.setDesiredState(CoralRollersState.L4_OUTTAKE), coralRollers).withTimeout(0.5).finallyDo(() -> {
              coralPivot.setDesiredState(CoralPivotState.MOVING);
              coralRollers.setDesiredState(CoralRollersState.STOPPED);
            }),
            new WaitUntilCommand(coralPivot::atDesiredState),
            new InstantCommand(() -> elevator.setDesiredState(ElevatorState.STARTING), elevator)
    );

    addRequirements(coralPivot, coralRollers, elevator);
  }
}