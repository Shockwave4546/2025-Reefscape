package org.dovershockwave.subsystems.coralpivot;

public enum CoralPivotState {
  STARTING(0.0, 0.0);

  public final double wristPositionRad;
  public final double biggerPivotPositionRad;

  CoralPivotState(double wristPositionRad, double biggerPivotPositionRad) {
    this.wristPositionRad = wristPositionRad;
    this.biggerPivotPositionRad = biggerPivotPositionRad;
  }
}
