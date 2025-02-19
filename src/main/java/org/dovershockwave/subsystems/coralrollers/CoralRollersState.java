package org.dovershockwave.subsystems.coralrollers;

public record CoralRollersState(double volts) {
  public static final CoralRollersState STOPPED = new CoralRollersState(0.0);
  public static final CoralRollersState INTAKE = new CoralRollersState(-6.0);
  public static final CoralRollersState OUTTAKE = new CoralRollersState(3.0);
}