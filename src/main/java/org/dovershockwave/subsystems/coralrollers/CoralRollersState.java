package org.dovershockwave.subsystems.coralrollers;

public record CoralRollersState(double volts) {
  public static final CoralRollersState STOPPED = new CoralRollersState(0.0);
  public static final CoralRollersState INTAKE = new CoralRollersState(-2.0);
  public static final CoralRollersState L1_OUTTAKE = new CoralRollersState(12.0);
  public static final CoralRollersState L2_OUTTAKE = new CoralRollersState(-12.0);
  public static final CoralRollersState L3_OUTTAKE = new CoralRollersState(-12.0);
  public static final CoralRollersState L4_OUTTAKE = new CoralRollersState(-12.0);
  public static final CoralRollersState INDEX = new CoralRollersState(1.5);

  public static final CoralRollersState KNOCKOFF_ALGAE = new CoralRollersState(12);
}