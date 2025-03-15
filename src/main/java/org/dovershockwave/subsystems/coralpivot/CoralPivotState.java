package org.dovershockwave.subsystems.coralpivot;

public record CoralPivotState(double wristPositionRad, double armPositionRad) {
  public static final CoralPivotState STARTING = new CoralPivotState(2.643, 2.05);
  public static final CoralPivotState MOVING = new CoralPivotState(2.25, 1.6);
  public static final CoralPivotState HUMAN_PLAYER = new CoralPivotState(0.25, 0.55);
  public static final CoralPivotState L1 = new CoralPivotState(-0.5, 0.225);
  public static final CoralPivotState L2_L3_OUTTAKE = new CoralPivotState(2.4, 0.5);
  public static final CoralPivotState L2 = new CoralPivotState(2.4, -0.125);
  public static final CoralPivotState L3 = new CoralPivotState(2.4, -0.125);
  public static final CoralPivotState L4 = new CoralPivotState(1.0, 0.75);

  public static final CoralPivotState ALGAE_KNOCK_OFF_INTERMEDIATE = new CoralPivotState(0, 0);
  public static final CoralPivotState ALGAE_KNOCK_OFF = new CoralPivotState(0, 0);
}