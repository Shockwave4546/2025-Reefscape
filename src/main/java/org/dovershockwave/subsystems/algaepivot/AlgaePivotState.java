package org.dovershockwave.subsystems.algaepivot;

public record AlgaePivotState(double positionRad) {
  public static final AlgaePivotState STARTING = new AlgaePivotState(0.0);
  public static final AlgaePivotState INTAKE = new AlgaePivotState(0.0);
}
