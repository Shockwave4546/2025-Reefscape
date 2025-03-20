package org.dovershockwave.subsystems.coralpivot;

public record CoralPivotState(double wristPositionRad, double armPositionRad) {
//  public static final CoralPivotState STARTING = new CoralPivotState(2.643, 2.05);
  public static final CoralPivotState MOVING = new CoralPivotState(2.75, -1.2);
  public static final CoralPivotState MOVING_UP = new CoralPivotState(2.25, 1.6);
  public static final CoralPivotState HUMAN_PLAYER = new CoralPivotState(2.07, -1.2);
  public static final CoralPivotState L1 = new CoralPivotState(0.7168, -0.29);
  public static final CoralPivotState L2_L3_OUTTAKE = new CoralPivotState(2.4, 0.5);
  public static final CoralPivotState L2 = new CoralPivotState(2.4594, 0.11417);
  public static final CoralPivotState L3 = new CoralPivotState(2.4594, 0.11417);
  public static final CoralPivotState L4 = new CoralPivotState(1.0, 0.75);

  public static final CoralPivotState SAFE_POSITION_AFTER_START_ONE = new CoralPivotState(1.41, 2.05); // Move wrist to safe spot
  public static final CoralPivotState SAFE_POSITION_AFTER_START_TWO = new CoralPivotState(1.41, 0.75); // Move arm forward
  public static final CoralPivotState SAFE_POSITION_AFTER_START_THREE = new CoralPivotState(2.75, 0.75); // Move wrist more in to moving
  public static final CoralPivotState SAFE_POSITION_AFTER_START_FOUR = new CoralPivotState(2.75, -1.2); // Move arm down to moving

  public static final CoralPivotState ALGAE_KNOCK_OFF_L2 = new CoralPivotState(0.2278, -0.066);
  public static final CoralPivotState ALGAE_KNOCK_OFF_L3 = new CoralPivotState(0.62, 0.2);
}