package org.dovershockwave.subsystems.coralpivot;

public record CoralPivotState(double wristPositionRad, double armPositionRad) {
  public static final CoralPivotState STARTING = new CoralPivotState(0.0, 0.0);
  public static final CoralPivotState MOVING = new CoralPivotState(2.25, 1.6);
  public static final CoralPivotState HUMAN_PLAYER = new CoralPivotState(-0.63, 1.25);
  public static final CoralPivotState L1 = new CoralPivotState(-0.5, 0.225);
  public static final CoralPivotState L2 = new CoralPivotState(2.25, -0.05);
  public static final CoralPivotState L3 = new CoralPivotState(2.25, -0.05);
  public static final CoralPivotState L4 = new CoralPivotState(1.0, 0.75);
}