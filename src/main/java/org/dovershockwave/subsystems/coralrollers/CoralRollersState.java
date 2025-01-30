package org.dovershockwave.subsystems.coralrollers;

public record CoralRollersState(String name, double velocityRadPerSec) {
  public static final CoralRollersState STOPPED = new CoralRollersState("Stopped", 0.0);
  public static final CoralRollersState INTAKE = new CoralRollersState("Intake", 0.0);
  public static final CoralRollersState OUTTAKE = new CoralRollersState("Outtake", 0.0);
}