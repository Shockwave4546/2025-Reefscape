package org.dovershockwave.subsystems.coralpivot;

public record CoralPivotState(double wristPositionRad, double armPositionRad) {
  public static final CoralPivotState STARTING = new CoralPivotState(0.0, 0.0);
  public static final CoralPivotState MOVING = new CoralPivotState(0.0, 0.0);
  public static final CoralPivotState HUMAN_PLAYER = new CoralPivotState(0.0, 0.0);
  public static final CoralPivotState L1 = new CoralPivotState(0.0, 0.0);
  public static final CoralPivotState L2 = new CoralPivotState(0.0, 0.0);
  public static final CoralPivotState L3 = new CoralPivotState(0.0, 0.0);
  public static final CoralPivotState L4 = new CoralPivotState(0.0, 0.0);
}