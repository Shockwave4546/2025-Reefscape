package org.dovershockwave.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.dovershockwave.subsystems.coralpivot.CoralPivotSubsystem;

public class FullAlgaeKnockoffCommand extends SequentialCommandGroup {
  public FullAlgaeKnockoffCommand(CoralPivotSubsystem coralPivot) {
    addCommands(
//            new InstantCommand(new InstantCommand())
    );

    addRequirements(coralPivot);
  }
}
