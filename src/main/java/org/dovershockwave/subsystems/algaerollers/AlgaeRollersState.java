package org.dovershockwave.subsystems.algaerollers;

public record AlgaeRollersState(String name, double velocityRadPerSec) {
  public static final AlgaeRollersState STOPPED = new AlgaeRollersState("Stopped", 0.0);
  public static final AlgaeRollersState INTAKE = new AlgaeRollersState("Intake", 0.0);
  public static final AlgaeRollersState OUTTAKE = new AlgaeRollersState("Outtake", 0.0);
}