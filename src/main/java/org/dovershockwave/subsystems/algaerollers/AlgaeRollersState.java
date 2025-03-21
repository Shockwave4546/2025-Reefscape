package org.dovershockwave.subsystems.algaerollers;

public record AlgaeRollersState(double volts) {
  public static final AlgaeRollersState STOPPED = new AlgaeRollersState(0.0);
  public static final AlgaeRollersState INTAKE = new AlgaeRollersState( 12);
  public static final AlgaeRollersState OUTTAKE = new AlgaeRollersState( -12);
  public static final AlgaeRollersState IDLE = new AlgaeRollersState(2);
}