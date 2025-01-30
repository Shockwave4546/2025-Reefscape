package org.dovershockwave.subsystems.algaerollers;

public enum AlgaeRollersState {
  STOPPED(0.0),
  INTAKE(0.0),
  OUTTAKE(0.0);

  public final double velocityRadPerSec;

  AlgaeRollersState(double velocityRadPerSec) {
    this.velocityRadPerSec = velocityRadPerSec;
  }
}
