package org.dovershockwave.subsystems.coralpivot;

// TODO: 2/1/2025 the bigger pivot's angles still need to be set. 
public record CoralPivotState(double wristPositionRad, double armPositionRad) {
  public static final CoralPivotState STARTING = new CoralPivotState(0, -1.2); // Good
  public static final CoralPivotState MOVING = new CoralPivotState(0, 1.6); // TODO: 2/18/2025
  public static final CoralPivotState HUMAN_PLAYER = new CoralPivotState(Math.PI / 2.0, 0.47); // TODO: 2/1/2025
  public static final CoralPivotState L1 = new CoralPivotState(Math.PI / 2.0, -0.15); // TODO: 2/1/2025
  public static final CoralPivotState L2 = new CoralPivotState(0.0, 0.0); // TODO: 2/1/2025  
  public static final CoralPivotState L3 = new CoralPivotState(0.0, 0.0); // TODO: 2/1/2025  
  public static final CoralPivotState L4 = new CoralPivotState(0.0, 0.75); // TODO: 2/1/2025
}