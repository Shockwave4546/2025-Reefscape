package org.dovershockwave.subsystems.coralrollers.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.dovershockwave.subsystems.coralrollers.CoralRollersConstants;
import org.dovershockwave.subsystems.coralrollers.CoralRollersState;
import org.dovershockwave.subsystems.coralrollers.CoralRollersSubsystem;

public class IntakeCoralCommand extends Command {
  public final CoralRollersSubsystem coralRollers;

  public IntakeCoralCommand(CoralRollersSubsystem coralRollers) {
    this.coralRollers = coralRollers;
    addRequirements(coralRollers);
  }

  @Override public void initialize() {
    coralRollers.setDesiredState(CoralRollersState.INTAKE);
  }

  @Override public void execute() {
    coralRollers.setDesiredState(CoralRollersState.INTAKE);
  }

  @Override public void end(boolean interrupted) {
    coralRollers.setDesiredState(CoralRollersState.STOPPED);
  }

  @Override public boolean isFinished() {
    return coralRollers.getLidarDistanceMeters() < CoralRollersConstants.CORAL_IN_POSITION_LIDAR_DISTANCE_MIN;
  }
}