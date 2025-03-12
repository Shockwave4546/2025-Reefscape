package org.dovershockwave.subsystems.algaepivot;

public record AlgaePivotState(double positionRad) {
  public static final AlgaePivotState STARTING = new AlgaePivotState(-0.97);
  public static final AlgaePivotState INTAKE_AFTER = new AlgaePivotState(0.1);
  public static final AlgaePivotState INTAKE = new AlgaePivotState(0.55);
  public static final AlgaePivotState OUTTAKE = new AlgaePivotState(0.2);
}
