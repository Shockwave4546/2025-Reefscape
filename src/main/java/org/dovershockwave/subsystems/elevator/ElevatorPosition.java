package org.dovershockwave.subsystems.elevator;

public enum ElevatorPosition {
  STARTING(0.0),
  L1(0.0),
  L2(0.0),
  L3(0.0),
  L4(0.0),
  HUMAN_PLAYER(0.0);

  public final double positionRad;

  ElevatorPosition(double positionRad) {
    this.positionRad = positionRad;
  }
}