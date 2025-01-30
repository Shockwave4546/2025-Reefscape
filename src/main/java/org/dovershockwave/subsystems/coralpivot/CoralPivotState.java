package org.dovershockwave.subsystems.coralpivot;

public record CoralPivotState(String name, double wristPositionRad, double biggerPivotPositionRad) {
  public static final CoralPivotState STARTING = new CoralPivotState("Starting", 0.0, 0.0);
}